import cv2
import numpy as np
import os
from PIL import Image


def get_limits(color):
    """
    Function to get the HSV color range for a given BGR color.

    :param color: A list representing the BGR color.
    :return: A tuple containing the lower and upper HSV limits for the color.
    """
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit


def detect_object(frame):
    """
    Function to detect an object of a specific color in a frame.

    :param frame: The frame in which to detect the object.
    :return: The original frame with the detected object highlighted and a boolean indicating if an object was detected.
    """
    orig = frame.copy()
    red = [0, 0, 255]  # red in BGR colorspace

    # blur the frame
    frame = cv2.GaussianBlur(frame, (17, 17), 0)

    # dilute the frame
    kernel = np.ones((5, 5), np.uint8)
    frame = cv2.dilate(frame, kernel, iterations=1)

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lowerLimit, upperLimit = get_limits(color=red)

    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

    mask_ = Image.fromarray(mask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox

        orig = cv2.rectangle(orig, (x1, y1), (x2, y2), (0, 255, 0), 5)

    return orig, (bbox is not None)


class CameraWrapper:
    """
    A class that wraps the functionality of a camera feed.
    """
    feed_url = 'http://192.168.0.3:5000/video_feed'  # URL of the video feed

    def __init__(self):
        """
        Constructor of the CameraWrapper class. It initializes the video capture from the feed URL.
        """
        self.cap = cv2.VideoCapture(self.feed_url)

    def is_visible(self, show=True):
        """
        Function to check if an object is visible in the camera feed.

        :param show: A boolean indicating whether to show the camera feed or not.
        :return: A boolean indicating whether an object is detected in the camera feed.
        """
        # Capture frame-by-frame
        ret, frame = self.cap.read()

        # If frame is read correctly ret is True
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            return None
        frame, is_detected = detect_object(frame)

        if show:
            # Display the resulting frame
            cv2.imshow('Camera Stream', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) == ord('q'):
                return

        return is_detected

    def camera_feed(self, detection=True):
        """
        Function to start the camera feed.

        :param detection: A boolean indicating whether to perform object detection in the camera feed or not.
        """
        if detection:
            self._camera_feed_with_detection()
        else:
            self._camera_feed_without_detection()

    def _camera_feed_with_detection(self):
        """
        Private function to start the camera feed with object detection.
        """
        while True:
            self.is_visible(show=True)

    def _camera_feed_without_detection(self):
        """
        Private function to start the camera feed without object detection.
        """
        while True:
            # Capture frame-by-frame
            ret, frame = self.cap.read()

            # If frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                return None

            # Display the resulting frame
            cv2.imshow('Camera Stream', frame)

            # Break the loop on 'q' key press
            if cv2.waitKey(1) == ord('q'):
                return


if __name__ == '__main__':
    camera = CameraWrapper()
    camera.camera_feed(detection=True)
