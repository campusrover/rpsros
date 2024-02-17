#!/usr/bin/env python
# 1. Subscribe to camera
# 2. Convert image to opencv
# 3. Use cv_aruco to recognize

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        # Process cv_image here
        cv2.imshow("Camera Image", cv_image)
        # cv2.waitKey(3)
        # ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
        # # Publish the image
        # image_pub.publish(ros_image)

if __name__ == '__main__':
    rospy.init_node('object_find', anonymous=True)
    rospy.Subscriber("/cv_camera/image_raw", Image, image_callback)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    rospy.spin()
