# Importing necessary Python libraries
import cv2
import numpy as np

class CustomKalmanFilter(object):
    """Custom implementation of the Kalman Filter class which tracks the estimated state of
    a system along with the variance or uncertainty of the estimate.
    The methods Predict and Correct implement the functionality of the filter.
    Reference: https://en.wikipedia.org/wiki/Kalman_filter
    Attributes: None
    """

    def __init__(self, initial_point):
        # Initialize the Kalman filter
        self.kalman_filter = cv2.KalmanFilter(4, 2)
        self.kalman_filter.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], np.float32)
        self.kalman_filter.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32)
        self.kalman_filter.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], np.float32) * 0.03

        # Set the initial state of the Kalman filter based on the initial point
        self.kalman_filter.statePost = np.array([[initial_point[0]], [initial_point[1]], [0], [0]], dtype=np.float32)

    def predict(self):
        """Predicts the next state of the system based on the previous state and system dynamics."""
        # Perform prediction using the Kalman filter
        predicted_state = tuple(self.kalman_filter.predict())
        return predicted_state

    def correct(self, measurement_point):
        """Updates the estimated state based on the given measurement point."""
        # Correct the estimated state using the provided measurement point
        corrected_state = self.kalman_filter.correct(np.array(measurement_point, dtype=np.float32))
        return corrected_state
