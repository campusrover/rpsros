#!/usr/bin/env python
# 1. Subscribe to camera
# 2. Convert image to opencv
# 3. Use cv_aruco to recognize

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

IMAGE_LOGGING = False

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict, parameters=arucoParams)
        # at least one Aruco detected. Flatten the ArUco IDs list
        if len(corners) > 0:
            ids = ids.flatten()
            image_logger(corners, ids)

def image_logger(corners, ids):
    if not IMAGE_LOGGING:
        return        
    # loop over the detected ArUCo corners
    for (markerCorner, markerID) in zip(corners, ids):
        # extract the marker corners (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        
        # draw the bounding box of the ArUCo detection
        cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the ArUco marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)
        # draw the ArUco marker ID on the image
        cv2.putText(cv_image, str(markerID),
            (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
            0.5, (0, 255, 0), 2)
    ros_image = bridge.cv2_to_imgmsg(cv_image, "bgr8")
    image_pub.publish(ros_image)

if __name__ == '__main__':
    rospy.init_node('object_find', anonymous=True)
    rospy.Subscriber("/cv_camera/image_raw", Image, image_callback)
    image_pub = rospy.Publisher('/sodacan/image_raw', Image, queue_size=10)
    tag_pub = rospy.Publisher('/sodacan/tag', , queue_size=10)
    rospy.spin()
