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
import bru_utils as bu

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
        #return (roll, pitch, yaw)
        return (pitch, roll, yaw)

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

    def rvec2RPY2(self, rvec: Tuple[float, float, float]) -> Tuple[float, float, float]:
        rotation_matrix = np.eye(4)
        rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(rvec))[0]
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
                rvec = rvecs[i][0]
                tvec = tvecs[i][0]

                # Trying which algo works correctly if any.
                # self.rpy0 = self.rvec2RPY0(rvec)
                # self.rpy1 = self.rvec2RPY1(rvec)
                self.rpy2 = self.rvec2RPY1(rvec)

                # Calculate distance using pythagorean theorem
                self.distance = sqrt(tvec[0] ** 2 + tvec[1] ** 2 + tvec[2] ** 2)
                self.bearing = atan2(tvec[0], tvec[2])

                # logstring = f"""d:{self.distance:5.2f}, b:{self.bearing:5.2f}, {self.log_string_create("rpy0", self.rpy0)}, \n{self.log_string_create("rpy1", self.rpy1)}, {self.log_string_create("rpy2", self.rpy2)}"""

                # Experiment is ongoing. THere are three algorithms for computing the pose from the markers. RPY1 and 2 give the exact same results. RPY0 is different. 
                # It is also unclear which of the three components are R-P or Y. So I have empircally measured to determine that when YAW the marker it is affecting the second
                # Component which according to me is Pitch. It has to do with orientation of the marker and the camera. For now I satisfy myself with empircally (trial and error) 
                # deciding which means what.
                self.aruco_message.data = [self.distance, self.bearing, self.rpy2[0]]
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
                self.text_on_image(cv_image)
                # ros_image = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
                self.image_pub.publish(ros_image)

    def text_on_image(self, img) -> NoReturn:
        line1 = f"""d:{self.distance:5.2f}, b:{self.bearing:5.2f}"""
        line2 = f"""{self.log_string_create("rpy2", self.rpy2)}"""

        font                   = cv2.FONT_HERSHEY_SIMPLEX
        font_scale              = 0.4
        font_color              = (0,0,0)
        thickness               = 1

        x, y = 150, 5  # Coordinates where you want the text
        line_spacing = 8

        # Get text size
        (text_width, text_height) = cv2.getTextSize(max(line1, line2), font, font_scale, thickness)[0]
        text_height += line_spacing

        # Create a white rectangle slightly larger than the text
        box_tl = (x, y)
        box_br = ((x + (text_width + line_spacing)), (y + text_height*3))
        cv2.rectangle(img, box_tl, box_br, (255, 255, 255), cv2.FILLED)

        # Add the text on top of the rectangle
        cv2.putText(img, line1, (x, y + text_height), font, font_scale, font_color, thickness) 
        cv2.putText(img, line2, (x, y + text_height*2), font, font_scale, font_color, thickness) 
        # cv2.putText(img, line3, (x, y + text_height*3), font, font_scale, font_color, thickness) 
        # cv2.putText(img, line4, (x, y + text_height*4), font, font_scale, font_color, thickness) 

    def log_string_create(self, title: str, tuple: Tuple[float, float, float])->str:
        return f"{title}{tuple[0]:5.2f},{tuple[1]:5.2f},{tuple[2]:5.2f}"

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
