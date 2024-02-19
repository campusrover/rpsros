#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

DEBUG = True

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
        self.bridge = CvBridge()
        self.marker_size = marker_size
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        if DEBUG:
            self.image_pub = rospy.Publisher("/sodacan/image_raw", Image, queue_size=10)

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, rejected = cv2.aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters
        )
        # If markers are detected
        if ids is not None:
            rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)            
            for tvec in tvecs:
                distance = np.linalg.norm(tvec)
                bearing = np.arctan2(tvec[0][0], tvec[0][2])  # Convert from radians to degrees if necessary
                print(f"Distance: {distance}, Bearing: {bearing} radians")

        if DEBUG:
            # Draw detected markers on the image
            cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)

    def run(self):
        rospy.Subscriber("/cv_camera/image_raw", Image, self.image_callback)
        rospy.spin()


if __name__ == "__main__":
    marker_detector = Detector(marker_size=0.04)
    marker_detector.run()




import cv2
import cv2.aruco as aruco
import numpy as np

# Assuming cameraMatrix and distCoeffs are known (from calibration)
cameraMatrix = # Your camera matrix
distCoeffs = # Your distortion coefficients

# Load the image
image = cv2.imread('path_to_your_image.jpg')

# Convert to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect markers
arucoDict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, arucoDict)

# If markers are detected
if ids is not None:
    # Estimate pose
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs)
    
    for tvec in tvecs:
        # Compute distance
        distance = np.linalg.norm(tvec)
        
        # Compute bearing (example for bearing in the xz plane)
        bearing = np.arctan2(tvec[0][0], tvec[0][2])  # Convert from radians to degrees if necessary
        
        print(f"Distance: {distance}, Bearing: {bearing} radians")

# Display the result
cv2.imshow('Image', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
