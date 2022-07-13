#!/usr/bin/env python3

import rospy
import tf2_ros
import signal
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2

def odom_cb(msg):
    global current_pose
    global pose_pub

    current_pose = msg.pose.position
    roll, pitch, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
    current_pose.theta = yaw
    pose_pub.publish(current_pose)

# Initialize this program as a node
rospy.loginfo("init")
rospy.init_node("line")
signal.signal(signal.SIGINT, self.shutdown)
odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
rate = rospy.Rate(10)
rospy.loginfo("move forward")
start = rospy.Time.now()
while not rospy.is_shutdown() and current_pose.x < 1.5:
    move = Twist()
    move.linear.x = 0.2
    move.angular.z = 0
    #movement_pub.publish(move)
    rospy.loginfo(current_pose)
    rate.sleep()
# rospy.loginfo("turn right")
# start_turn = rospy.Time.now()
# while not rospy.is_shutdown() and current_pose.theta > -pi:
#     move = Twist()
#     move.linear.x = 0
#     move.angular.z = 0.3
#     movement_pub.publish(move)
#     rospy.spinonce()
#     rate.sleep()
# while not rospy.is_shutdown() and current_pose.x > -1.5:
#     move = Twist()
#     move.linear.x = 0.2
#     move.angular.z = 0
#     movement_pub.publish(move)
#     rospy.spinonce()
#     rate.sleep()
# while not rospy.is_shutdown():
#     move = Twist()
#     move.linear.x = 0
#     move.angular.z = 0
#     movement_pub.publish(move)
#     rospy.spinonce()
#     rate.sleep()
