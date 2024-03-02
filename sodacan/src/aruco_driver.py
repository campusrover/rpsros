#!/usr/bin/env python
import rospy
import actionlib
import tf2_ros

from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf2_ros import Buffer, TransformListener

# https://github.com/husky/husky/blob/indigo-devel/husky_navigation/launch/move_base_mapless_demo.launch


class ArucoNavigator:
    def __init__(self):
        rospy.init_node("aruco_navigator")

        self.aruco_frame_id = "fiducial_0"
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.move_base_client = actionlib.SimpleActionClient(
            "move_base", MoveBaseAction
        )
        self.move_base_client.wait_for_server()

        self.navigate = False
        self.rate = rospy.Rate(10)
        self.update_goal()

    def update_goal(self):
        try:
            tag_trans = self.tf_buffer.lookup_transform(
                "base_link", self.aruco_frame_id, rospy.Time(0)
            )

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.aruco_frame_id
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = tag_trans.transform.translation.x
            goal.target_pose.pose.position.y = tag_trans.transform.translation.y

            goal.target_pose.pose.orientation.w = 1.0

            self.move_base_client.send_goal(goal)

        except:
            pass

    def run(self):
        while not rospy.is_shutdown():
            if self.navigate:
                self.update_goal()
            self.rate.sleep()

    def stop(self):
        self.cmd_vel_pub.publish(Twist())
        self.navigate = False

if __name__ == "__main__":
    nav = ArucoNavigator()
    nav.navigate = True
    nav.run()
