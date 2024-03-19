#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class ArucoMoveBase():
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('set_tf_as_navigation_goal')

    def set_tf_as_navigation_goal(self, from_frame, to_frame):
        # Create a client for the move_base action
        move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        move_base_client.wait_for_server()

        # Create a TF buffer and listener
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Get the transform from from_frame to to_frame
        try:
            transform = tf_buffer.lookup_transform(from_frame, to_frame, rospy.Time.now(), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Cannot get transform {from_frame}' to {to_frame} (error: {str(e)})")
            return

        # Create a PoseStamped message with the transform
        goal_pose = PoseStamped()
        goal_pose.header = transform.header
        goal_pose.pose.position = transform.transform.translation
        goal_pose.pose.orientation = transform.transform.rotation

        # Transform the goal pose from 'tfname' frame to 'map' frame
        try:
            tf_buffer.transform(goal_pose, 'map', rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn(f"Cannot get transform {from_frame}' to {to_frame} (error: {str(e)})")
            return

        # Create a move_base goal with the transformed goal pose
        goal = MoveBaseGoal()
        goal.target_pose = goal_pose

        # Send the goal to the move_base action
        move_base_client.send_goal(goal)
        move_base_client.wait_for_result()

        if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation goal set successfully!")
        else:
            rospy.logwarn("Failed to set navigation goal.")

if __name__ == '__main__':
    an = ArucoMoveBase()
    an.set_tf_as_navigation_goal("fiducial_0")