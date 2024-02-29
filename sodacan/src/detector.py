#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float64MultiArray
import numpy as np
from math import atan2, sqrt, sin, cos
from typing import NoReturn, Tuple
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

    def rvec2RPY0(self, rvec: Tuple[float, float, float]) -> Tuple[float, float, float]:
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
        return roll, pitch, yaw

    def rvec2RPY1(self, rvec: Tuple[float, float, float]) -> Tuple[float, float, float]:
        rmat, _ = cv2.Rodrigues(rvec)
        sy = sqrt(rmat[0, 0] * rmat[0, 0] + rmat[1, 0] * rmat[1, 0])
        singular = sy < 1e-6
        if not singular:
            x = atan2(rmat[2, 1], rmat[2, 2])
            y = atan2(-rmat[2, 0], sy)
            z = atan2(rmat[1, 0], rmat[0, 0])
        else:
            x = atan2(-rmat[1, 2], rmat[1, 1])
            y = atan2(-rmat[2, 0], sy)
            z = 0
        return (x, y, z)

    def rec2RPY2(self, rvec: Tuple[float, float, float]) -> Tuple[float, float, float]:
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec[0]))[0]
        r = R.from_matrix(rotation_matrix[0:3, 0:3])
        quat = r.as_quat()
        euler_rotation = euler_from_quaternion(quat)
        return euler_rotation

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = self.prepare_marker_image(cv_image)
        corners, ids, _ = cv2.aruco.detectMarkers(
            cv_image, self.dictionary, parameters=self.parameters
        )
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.camera_matrix, self.dist_coeffs
        )

        # If markers are detected
        if ids is not None:
            for i in range(len(corners)):
                rvec = rvecs[i]
                tvec = tvecs[i]

                # Trying which algo works correctly if any.
                rpy0 = self.rvec2RPY0(rvec)
                rpy1 = self.rvec2RPY1(rvec)
                rpy2 = self.rvec2RPY2(rvec)

                # Calculate distance using pythagorean theorem
                distance = sqrt(tvec[0] ** 2 + tvec[1] ** 2 + tvec[2] ** 2)
                bearing = atan2(tvec[0], tvec[2])

                logstring = f"d:{distance:5.2f}, b:{bearing:5.2f}, rpy0:{rpy0}, rpy1:{rpy1}, rpy2:{rpy2}")

                self.aruco_message.data = [distance, bearing]
                self.aruco_pub.publish(self.aruco_message)

            if DEBUG:
                # Draw detected markers on the image
                for id in ids:
                    cv2.drawFrameAxes(
                        cv_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs,
                        tvecs,
                        self.marker_size,
                    )
                cv_image = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                self.text_on_image(logstring, cv_image)
                # ros_image = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(ros_image)

    def text_on_image(self, text: str, image) -> NoReturn:
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = [30,30]
        fontScale              = 0.7
        fontColor              = (255,255,255)
        lineType               = 2
        cv2.putText(image, text, tuple(bottomLeftCornerOfText), font, fontScale, fontColor, lineType)


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
