#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

DEBUG = False

class Detector:
    """
    A class to detect ArUco markers using OpenCV in a ROS environment.

    Attributes:
    - bridge (CvBridge): A bridge to convert ROS Image messages to OpenCV images.
    - marker_size (float): The size of the ArUco marker in meters.
    - dictionary (cv2.aruco.Dictionary): The ArUco marker dictionary.
    - parameters (cv2.aruco.DetectorParameters): The parameters for marker detection.
    """

    def __init__(self, marker_size: float):
        # Initialize the ROS node
        rospy.init_node("aruco_marker_detector")

        # Create a CvBridge object to convert ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Set the marker size
        self.marker_size = marker_size

        # Load the ArUco marker dictionary
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        # Set the parameters for marker detection
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.image_pub = rospy.Publisher("/sodacan/image_raw", Image, queue_size=10)

    def image_callback(self, msg: Image):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Detect ArUco markers in the image
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters
        )

        # Draw detected markers on the image
        cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Display the image
        if DEBUG:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)

    def run(self):
        rospy.Subscriber("/cv_camera/image_raw", Image, self.image_callback)
        rospy.spin()


if __name__ == "__main__":
    marker_detector = Detector(marker_size=0.04)
    marker_detector.run()
