import numpy as np
from kalman_filter import KalmanFilter

class TrackedObject:
    """Class to represent a tracked object."""

    def __init__(self, initial_position, track_id):
        """Initialize variables used by the TrackedObject class.
        
        Args:
            initial_position: Initial position of the tracked object.
            track_id: Unique identifier for the tracked object.
        """
        self.track_id = track_id
        self.kalman_filter = KalmanFilter(initial_position)
        self.prediction = np.asarray(initial_position)
        self.skipped_frames = 0
        self.last_detection_assignment = 0
        self.age = 0
        self.trace = []

    def update_prediction(self):
        """Update the prediction of the tracked object."""
        self.prediction = self.kalman_filter.predict()

    def correct_prediction(self, measurement):
        """Correct the prediction based on the measurement.
        
        Args:
            measurement: Detected measurement used to correct the prediction.
        """
        self.prediction = self.kalman_filter.correct(measurement)

    def update_trace(self):
        """Update the trace history of the tracked object."""
        if len(self.trace) > 100:
            self.trace = self.trace[-100:]
        self.trace.append(self.prediction)

    def increment_age(self):
        """Increment the age of the tracked object."""
        self.age += 1

    def increment_skipped_frames(self):
        """Increment the number of frames skipped without detection."""
        self.skipped_frames += 1
