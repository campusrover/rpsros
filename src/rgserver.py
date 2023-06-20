#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from bru_utils import turn_to_target, wait_for_simulator, poses_close, calc_distance, normalize_angle
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2, isclose, cos, sin, degrees, radians



class RoboGym:
    def __init__(self):
        self.pub_cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.target = Pose2D(1, 1, 0)
        self.odom_pose = Pose2D(0, 0, 0)
        self.required_turn_angle = 0
    
    def odom_cb(self, msg):
        self.oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([self.oreuler.x, self.oreuler.y, self.oreuler.z, self.oreuler.w])
        self.odom_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.required_turn_angle = turn_to_target(self.odom_pose.theta, self.odom_pose.x, self.odom_pose.y, self.target.x, self.target.y)

    def run(self):
        self.twist = Twist()
        self.rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            print(f"target: {self.target.x:2.2f}, {self.target.y:2.2f} curr: {self.odom_pose.x:2.2f} {self.odom_pose.y:2.2f})")
            print(f"dist: {calc_distance(self.odom_pose, self.target):2.2f}, turn: {degrees(normalize_angle(self.required_turn_angle)):2.2f}")
            if calc_distance(self.odom_pose, self.target) > 0.1:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.2 if normalize_angle(self.required_turn_angle) > 0 else -0.2
            else:
                self.twist.linear.x = self.twist.angular.z = 0
                self.pub_cmd_vel.publish(self.twist)
                return
            self.pub_cmd_vel.publish(self.twist)
            self.rate.sleep()
            
# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    wait_for_simulator()
    try:
        rg = RoboGym()
        rg.run()
        print("running")
    except rospy.ROSInterruptException:
        pass
