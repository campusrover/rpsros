#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
import numpy as np
from math import atan2, sqrt, sin, cos
from typing import Tuple
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion

DEBUG = True


class Detector:
    def __init__(self, marker_size: float):
        rospy.init_node("detector")
        self.bridge = CvBridge()
        self.marker_size = marker_size
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_needed = True
        self.aruco_message = Float64MultiArray()
        self.aruco_pub = None
        if DEBUG:
            self.image_pub = rospy.Publisher("/sodacan/image_raw", Image, queue_size=10)

    def prepare_marker_image(self, img):
        # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


        # # Maximize contrast with adaptive thresholding
        # thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        return img

    def get_metrics_from_pose(
        self, rvec: Tuple[float, float, float], tvec: Tuple[float, float, float]
    ) -> Tuple[float, float, float, float, float]:

        # rvec - Rotation Vector
        #   This is a 3x1 vector (array with 3 numbers) that contains the rotation of the marker/object in axis-angle representation
        #   The 3 numbers represent rotation around X, Y, and Z axes respectively
        #   Units are in radians
        #   This compactly represents the 3D orientation of the fiducial marker

        # tvec - Translation Vector
        #   This is also a 3x1 vector (3 numbers)
        #   Contains the 3D position of the fiducial marker center point with respect to the camera
        #   The units are usually meters
        #   The 3 numbers represent the X, Y and Z displacement of the marker from the camera origin

        # Calculate distance using pythagorean theorem
        # sqrt(x^2 + y^2 + z^2)
        distance = sqrt(tvec[0] ** 2 + tvec[1] ** 2 + tvec[2] ** 2)

        # Bearing using atan2(x / z)
        # Returns angle between x-axis and (x, z) point
        bearing = atan2(tvec[0], tvec[2])

        # Roll from rotation vector
        # cos(roll) = sqrt(1 - sin(theta)^2) using trig identity
        theta = rvec[1]
        cos_roll = sqrt(1 - sin(theta) ** 2)

        # sin(roll) derived using trig identities
        sin_roll = rvec[2] / cos_roll
        roll = atan2(sin_roll, cos_roll)

        # Pitch using trig identities and atan2
        x = rvec[0] / sqrt(1 - sin(roll) ** 2)
        pitch = atan2(sin(x), cos(x))

        # Yaw using trig identities and atan2
        y = rvec[1] / sqrt(1 - sin(x) ** 2)
        yaw = atan2(sin(y), cos(y))

        return distance, bearing, roll, pitch, yaw

    def compute_pose(self, marker_ids, corners, aruco_marker_side_length, mtx, dst):
        # Algorithm from https://automaticaddison.com/how-to-perform-pose-estimation-using-an-aruco-marker/
        # Get the rotation and translation vectors
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
            corners, aruco_marker_side_length, mtx, dst
        )

        # The pose of the marker is with respect to the camera lens frame.
        # Imagine you are looking through the camera viewfinder,
        # the camera lens frame's:
        # x-axis points to the right
        # y-axis points straight down towards your toes
        # z-axis points straight ahead away from your eye, out of the camera
        for i, marker_id in enumerate(marker_ids):

            # Store the translation (i.e. position) information
            translation = tvecs[i][0]

            # Store the rotation information
            rotation_matrix = np.eye(4)
            rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvecs[i][0]))[0]
            r = R.from_matrix(rotation_matrix[0:3, 0:3])
            quat = r.as_quat()

            # Euler angle format in radians
            euler_rotation = euler_from_quaternion(quat)
            return (
                translation,
                euler_rotation,
            )

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = self.prepare_marker_image(cv_image)
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters
        )

        # If markers are detected
        if ids is not None:
            trans, rot = self.compute_pose(
                ids, corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            for i in range(len(corners)):
                distance, bearing, roll, pitch, yaw = self.get_metrics_from_pose(
                    rvec=rvecs[i][0], tvec=tvecs[i][0]
                )
                rospy.logdebug(
                    f"d:{distance:5.2f}, b:{bearing:5.2f}, rpy:({roll:6.2f},{pitch:6.2f},{yaw:6.2f}) rpy:({rot[0]:6.2f},{rot[1]:6.2f},{rot[2]:6.2f}) trans:({trans[0]:6.2f},{trans[1]:6.2f},{trans[2]:6.2f})"
                )
                self.aruco_message.data = [
                    distance,
                    bearing,
                    roll,
                    pitch,
                    yaw,
                    rot[0],
                    rot[1],
                    rot[2]
                ]
                self.aruco_pub.publish(self.aruco_message)

            if DEBUG:
                # Draw detected markers on the image
                for id in ids:
                    cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvecs, tvecs, self.marker_size)
                cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                # ros_image = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(ros_image)

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_info_needed:
            self.camera_matrix = np.array(msg.K).reshape((3, 3))
            self.dist_coeffs = msg.D
            self.camera_info_needed = False

    def run(self):
        rospy.Subscriber("/cv_camera/image_raw", Image, self.image_callback)
        rospy.Subscriber(
            "/cv_camera/camera_info", CameraInfo, self.camera_info_callback
        )
        self.aruco_pub = rospy.Publisher("/aruco", Float64MultiArray, queue_size=1)
        rospy.loginfo("Detector running...")
        rospy.spin()


if __name__ == "__main__":
    marker_detector = Detector(marker_size=0.045)
    marker_detector.run()
