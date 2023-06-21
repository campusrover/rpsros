#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from bru_utils import turn_to_target, wait_for_simulator, poses_close, calc_distance, normalize_angle, sigmoid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2, isclose, cos, sin, degrees, radians, ceil
from rgparser import CommandInterface

class RoboGym:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.target = Pose2D(1, 1, 0)
        self.odom_pose = Pose2D(0, 0, 0)
        self.required_turn_angle = 0

    def log(self, msg: str):
        if self.values["log"] == 1:
            print(msg)

    def safe_publish_cmd_vel(self):
        safe = Twist()
        #print(rg.values["max_lin"], rg.values["max_ang"])
        #safe.linear.x = min(self.twist.linear.x,rg.values["max_lin"])
        #safe.angular.z = min(self.twist.linear.x,rg.values["max_ang"])
        #self.log(f"lin: {safe.linear.x:2.2f}, ang: {safe.angular.z:2.2f}")
        self.cmd_vel_pub.publish(self.twist)

    def odom_cb(self, msg):
        self.oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([self.oreuler.x, self.oreuler.y, self.oreuler.z, self.oreuler.w])
        self.odom_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.required_turn_angle = turn_to_target(self.odom_pose.theta, self.odom_pose.x, self.odom_pose.y, self.target.x, self.target.y)

    def goto(self):
    # This can be improved by applying a sigmoid function.
        self.twist = Twist()
        self.rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            distance = calc_distance(self.odom_pose, self.target)
            abs_required_turn = abs(self.required_turn_angle)
            if  distance > 0.1:
                sig_turn = sigmoid(abs_required_turn, 1, 0.75)
                self.twist.linear.x = 0 if abs_required_turn > 0.3 else sigmoid(distance, 1, 0.5)
                self.twist.angular.z = sig_turn if normalize_angle(self.required_turn_angle) > 0 else -sig_turn
                self.log(f"target: {self.target.x:2.2f}, {self.target.y:2.2f} curr: {self.odom_pose.x:2.2f} {self.odom_pose.y:2.2f})" +
                        f", dist: {distance:2.2f}, turn: {degrees(normalize_angle(self.required_turn_angle)):2.2f}" +
                        f", lin: {self.twist.linear.x:2.2f}, ang: {self.twist.angular.z:2.2f}")

                self.safe_publish_cmd_vel()
            else:
                self.twist.linear.x = self.twist.angular.z = 0
                self.safe_publish_cmd_vel()
                return
            self.rate.sleep()

    def stop(self):
        self.twist = Twist()
        self.safe_publish_cmd_vel()

            
# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    wait_for_simulator()
    cp = CommandInterface()
    rg = RoboGym()
    while not rospy.is_shutdown():
        try:
            command, message, result, values = cp.command()
            rg.values = values 
            if command == "quit":
                break
            elif command == "goto":
                rg.target = Pose2D(values["x"], values["y"], 0)
                rg.goto()
            elif command == "stop":
                rg.stop()
        except rospy.ROSInterruptException:
            pass

