#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from bru_utils import turn_to_target, 

class RoboGym:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.target = Pose2D(0, 1, 0)
    
    def odom_cb(self, msg):
        self.oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
        self.odom_pose = Pose2D(msg.pose.position.x, msg.pose.position.y, yaw)
        self.required_turn_angle = turn_to_target(self.odom_pose.yaw, self.odom_pose.x, self.odom_pose.y, self.target.x, self.target.y)

    def run(self):
        twist = Twist()
        while not rospy.is_shutdown():

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    try:
        RoboGym().run()
    except rospy.ROSInterruptException:
        pass
