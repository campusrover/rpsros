#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from bru_utils import turn_to_target, wait_for_simulator, poses_close, calc_distance, normalize_angle, sigmoid, offset_point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2, isclose, cos, sin, degrees, radians, ceil
from rgparser import CommandInterface
from rgmonitor import Monitor

class RoboGym:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.target = Pose2D(1, 1, 0)
        self.odom_pose = Pose2D(0, 0, 0)
        self.required_turn_angle = 0
        self.monitor = Monitor()

    def log(self, msg: str):
        if self.values["log"] == 1:
            print(msg)

    def safe_publish_cmd_vel(self):
        safe = Twist()
        safe.linear.x = min(self.twist.linear.x,rg.values["max_lin"])
        safe.angular.z = min(self.twist.linear.x,rg.values["max_ang"])
        self.log(f"target: {self.target.x:2.2f}, {self.target.y:2.2f} curr: {self.odom_pose.x:2.2f} {self.odom_pose.y:2.2f})" +
                f", dist: {self.distance:2.2f}, turn: {degrees(normalize_angle(self.required_turn_angle)):2.2f}" +
                f", lin: {self.twist.linear.x:2.2f}, ang: {self.twist.angular.z:2.2f}")
        self.cmd_vel_pub.publish(self.twist)

    def odom_cb(self, msg):
        self.oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([self.oreuler.x, self.oreuler.y, self.oreuler.z, self.oreuler.w])
        self.odom_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.required_turn_angle = turn_to_target(self.odom_pose.theta, self.odom_pose.x, self.odom_pose.y, self.target.x, self.target.y)

    def goto(self):
        self.twist = Twist()
        self.target = Pose2D(self.values["x"], self.values["y"], 0)
        self.rate = rospy.Rate(10)
        self.monitor.say("Commanded to goto")
        while not rospy.is_shutdown():
            self.distance = calc_distance(self.odom_pose, self.target)
            abs_required_turn = abs(degrees(normalize_angle(self.required_turn_angle)))
            if  self.distance > 0.1:
                sig_turn = sigmoid(abs_required_turn, 0.5, rg.values["target_ang"])
                self.twist.linear.x = 0 if abs_required_turn > 20 else sigmoid(self.distance, 0.5, rg.values["target_lin"])
                self.twist.angular.z = sig_turn if normalize_angle(self.required_turn_angle) > 0 else -sig_turn
                self.safe_publish_cmd_vel()
            else:
                self.twist.linear.x = self.twist.angular.z = 0
                self.safe_publish_cmd_vel()
                return
            self.rate.sleep()

    def stop(self):
        self.twist = Twist()
        self.distance = 0
        self.monitor.say("Commanded to stop")
        self.safe_publish_cmd_vel()

    def move(self):
        self.twist = Twist()
        self.rate = rospy.Rate(10)
        forward_speed =  self.values["forward_speed"]
        self.distance = self.values["distance"]
        self.target = offset_point(self.odom_pose, self.distance)
        self.monitor.say("Commanded to move")
        while not rospy.is_shutdown():
            self.distance = abs(calc_distance(self.odom_pose, self.target))
            if  self.distance > 0.5:
                self.twist.linear.x = forward_speed
            elif self.distance > 0.05:
                self.twist.linear.x = 0.1 if forward_speed > 0 else -0.1
            else:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
            self.safe_publish_cmd_vel()
            if self.distance < 0.1: return
            self.rate.sleep()
    
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
                rg.goto()
            elif command == "stop":
                rg.stop()
            elif command == "move":
                rg.move()
        except rospy.ROSInterruptException:
            continue

