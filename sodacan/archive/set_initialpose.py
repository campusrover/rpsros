#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('initial_pose_publisher')

# Create a publisher for the initial pose
initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

# Create a PoseWithCovarianceStamped message
initial_pose_msg = PoseWithCovarianceStamped()

# Set the header frame ID to the map frame
initial_pose_msg.header.frame_id = 'map'

# Set the robot's position and orientation
initial_pose_msg.pose.pose.position.x = 0.0
initial_pose_msg.pose.pose.position.y = 0.0
initial_pose_msg.pose.pose.position.z = 0.0

initial_pose_msg.pose.pose.orientation.x = 0.0
initial_pose_msg.pose.pose.orientation.y = 0.0
initial_pose_msg.pose.pose.orientation.z = 0.0
initial_pose_msg.pose.pose.orientation.w = 1.0

# Publish the initial pose
initial_pose_pub.publish(initial_pose_msg)

rospy.spin()