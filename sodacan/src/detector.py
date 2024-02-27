#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
import numpy as np

DEBUG = False

class Detector:
    def __init__(self, marker_size: float):
        rospy.init_node("detector")
        self.bridge = CvBridge()
        self.marker_size = marker_size
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_needed = True
        self.aruco_message = Float64MultiArray()
        self.aruco_pub = None
        if DEBUG:
            self.image_pub = rospy.Publisher("/sodacan/image_raw",
                                             Image,
                                             queue_size=10)

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters)
        # If markers are detected
        if ids is not None:
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for tvec in tvecs:
                # Compute distance and bearing
                distance = np.linalg.norm(tvec)
                bearing = np.arctan2(
                    tvec[0][0],
                    tvec[0][2])  # Convert from radians to degrees if necessary
                rospy.loginfo(f"Distance: {distance:.2f}, Bearing: {bearing:.2f} radians")
                self.aruco_message.data = [distance, bearing]
                self.aruco_pub.publish(self.aruco_message)

        if DEBUG:
            # Draw detected markers on the image
            cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info_needed:
            self.camera_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = msg.D
            self.camera_info_needed = False

    def run(self):
        rospy.Subscriber("/cv_camera/image_raw", Image, self.image_callback)
        rospy.Subscriber("/cv_camera/camera_info", CameraInfo, self.camera_info_callback)
        self.aruco_pub = rospy.Publisher('/aruco', Float64MultiArray, queue_size=1)
        rospy.loginfo("Detector running...")
        rospy.spin()

if __name__ == "__main__":
    marker_detector = Detector(marker_size=0.045)
    marker_detector.run()
    