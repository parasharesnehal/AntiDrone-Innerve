import numpy as np
from scipy.optimize import linear_sum_assignment

from tracking_module import TrackedObject

# Adjusted threshold for creating a new track
minimum_detection_confidence_threshold = 20

class ObjectTracker:
    def __init__(self, distance_threshold, max_frames_to_skip, max_trace_length,
                 track_id_count):
        """Initialize variables used by the ObjectTracker class.
        Args:
            distance_threshold: Threshold for deleting tracks and creating new ones.
            max_frames_to_skip: Maximum allowed frames to be skipped for undetected tracks.
            max_trace_length: Maximum length of trace history.
            track_id_count: Identification of each tracked object.
        """
        self.distance_threshold = distance_threshold
        self.max_frames_to_skip = max_frames_to_skip
        self.max_trace_length = max_trace_length
        self.tracks = []
        self.track_id_count = track_id_count

    def update(self, detections_confidence):
        """Update tracks using the detected centroids of objects to be tracked.
        Args:
            detections_confidence: Detected centroids and their confidence scores.
        """
        detections = [np.round(np.array([[d[1]], [d[2]]])) for d in detections_confidence]

        # Create tracks if no tracks are found
        if not self.tracks:
            if detections and detections_confidence[0][0] > minimum_detection_confidence_threshold:
                track = TrackedObject(detections[0], self.track_id_count)
                self.track_id_count += 1
                self.tracks.append(track)

        # Calculate cost using sum of square distance between predicted and detected centroids
        n_tracks = len(self.tracks)
        n_detections = len(detections)
        cost = np.zeros(shape=(n_tracks, n_detections))

        for i, track in enumerate(self.tracks):
            track.age += 1
            for j, detection in enumerate(detections):
                diff = track.prediction - detection
                distance = np.sqrt(diff[0][0] ** 2 + diff[1][0] ** 2)
                cost[i][j] = distance

        # Average the squared error
        cost = 0.5 * cost

        # Assign detected measurements to predicted tracks using the Hungarian Algorithm
        row_ind, col_ind = linear_sum_assignment(cost)
        assignment = [-1] * n_tracks

        for i, row in enumerate(row_ind):
            assignment[row] = col_ind[i]

        # Identify tracks with no assignment
        unassigned_tracks = []

        for i, assignment_index in enumerate(assignment):
            if assignment_index != -1:
                # Check distance threshold, unassign if cost is too high
                if cost[i][assignment_index] > self.distance_threshold:
                    assignment[i] = -1
                    unassigned_tracks.append(i)
            else:
                self.tracks[i].skipped_frames += 1

        # Remove tracks not detected for a long time
        del_tracks = [i for i, track in enumerate(self.tracks) if track.skipped_frames > self.max_frames_to_skip]

        for track_index in del_tracks:
            if track_index < len(self.tracks):
                del self.tracks[track_index]
                del assignment[track_index]
            else:
                print("ERROR: Track index is greater than the length of tracks")

        # Update KalmanFilter state, lastResults, and tracks trace
        for i, assignment_index in enumerate(assignment):
            self.tracks[i].prediction = self.tracks[i].KF.predict()

            self.tracks[i].last_detection_assignment = None
            if assignment_index != -1:
                self.tracks[i].skipped_frames = 0
                self.tracks[i].prediction = self.tracks[i].KF.correct(detections[assignment_index])
                self.tracks[i].last_detection_assignment = assignment_index

            if len(self.tracks[i].trace) > self.max_trace_length:
                del self.tracks[i].trace[:len(self.tracks[i].trace) - self.max_trace_length]

            self.tracks[i].KF.lastResult = self.tracks[i].prediction
