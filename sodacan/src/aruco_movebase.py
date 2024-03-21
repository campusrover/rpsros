#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PointStamped
from actionlib_msgs.msg import GoalStatusArray
from tf.transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import Quaternion
import tf
import math


class ArucoMoveBase():
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('arucomovebase')

    def set_initial_pose(self):
        initial_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.pose.pose.position.x = 0.0
        initial_pose_msg.pose.pose.position.y = 0.0
        initial_pose_msg.pose.pose.position.z = 0.0
        initial_pose_msg.pose.pose.orientation.x = 0.0
        initial_pose_msg.pose.pose.orientation.y = 0.0
        initial_pose_msg.pose.pose.orientation.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0
        # Publish the initial pose
        initial_pose_pub.publish(initial_pose_msg)

    def get_pose_in_tf(self, source_pose, source_tf, target_tf):
        listener = tf.TransformListener()
        listener.waitForTransform(target_tf, source_tf, rospy.Time(), rospy.Duration(1.0))
        transform_time = rospy.Time(0)
        try:
            target_pose = listener.transformPose(target_tf, source_pose)
            rotation_quaternion = quaternion_from_euler(0, 0, 0)
            o = target_pose.pose.orientation
            unrotated_quaternion = [o.x, o.y, o.z, o.w]
            rotated_quaternion = quaternion_multiply(unrotated_quaternion, rotation_quaternion)
            target_pose.pose.orientation.x = rotated_quaternion[0]
            target_pose.pose.orientation.y = rotated_quaternion[1]
            target_pose.pose.orientation.z = rotated_quaternion[2]
            target_pose.pose.orientation.w = rotated_quaternion[3]
            rospy.loginfo("Converted coordinate: %s", target_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Failed to convert coordinate")
        return target_pose

    def set_pose_as_navigation_goal(self, goal_pose):
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()
        print(move_base_client.get_state())
        if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation goal set successfully!")
        else:
            rospy.logwarn("Failed to set navigation goal.")

    def set_pose_as_navigation_goal1(self, goal_pose):
        goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        goal_pub.publish(goal_pose)


    def check_navigation_status(self):    
        status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.callback)
        rospy.spin()

    def callback(self,status_msg):
        if len(status_msg.status_list) > 0:
            status = status_msg.status_list[0].status
            if status == 1:  # Goal is currently being processed
                rospy.loginfo("Robot is currently moving to the goal.")
            elif status == 3:  # Goal has been reached
                rospy.loginfo("Robot has reached the goal.")
            elif status == 4:  # Goal was aborted
                rospy.logwarn("Robot failed to reach the goal.")
            elif status == 5:  # Goal was rejected
                rospy.logwarn("Robot goal was rejected.")


if __name__ == '__main__':
    an = ArucoMoveBase()
    an.set_initial_pose()

    source_tf = "fiducial_0"
    target_tf = "map"
    source_pose = PoseStamped()
    source_pose.header.frame_id = source_tf
    source_pose.pose.position.x = 0.0
    source_pose.pose.position.y = 0.0
    source_pose.pose.position.z = 0.0
    q = quaternion_from_euler(0, 0, 0)
    source_pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    goal_pose = an.get_pose_in_tf(source_pose, source_tf, target_tf)
    an.set_pose_as_navigation_goal(goal_pose)
    an.check_navigation_status()

