#!/usr/bin/env python3
"""This module puts up a command prompt and gives the user control by commands of the robot"""

from math import degrees
import rospy
from geometry_msgs.msg import Twist, Pose2D
from bru_utils import turn_to_target, wait_for_simulator, calc_distance, normalize_angle, sigmoid, offset_point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
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
        self.twist = None
        self.distance = None
        self.values = None
        self.exit = False

    def log(self, msg: str):
        """print out log messages if the log variable is set to 1"""
        if self.values["log"] == 1:
            print(msg)

    def safe_publish_cmd_vel(self):
        """Publish cmd_vel from self.twist, after applying maximum speed and rotation constraints.
        Note that this will probably move to a new safe_cmd_vel node"""
        safe = Twist()
        safe.linear.x = min(self.twist.linear.x,rg.values["max_lin"])
        safe.angular.z = min(self.twist.linear.x,rg.values["max_ang"])
        self.log(f"target: {self.target.x:2.2f}, {self.target.y:2.2f}" + 
                f" curr: {self.odom_pose.x:2.2f} {self.odom_pose.y:2.2f})" +
                f", dist: {self.distance:2.2f}, turn: {degrees(normalize_angle(self.required_turn_angle)):2.2f}" +
                f", lin: {self.twist.linear.x:2.2f}, ang: {self.twist.angular.z:2.2f}")
        self.cmd_vel_pub.publish(self.twist)

    def odom_cb(self, msg: str):
        """ROS callback for Odometry Message"""
        oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
        self.odom_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.required_turn_angle = turn_to_target(self.odom_pose.theta, self.odom_pose.x, self.odom_pose.y, self.target.x, self.target.y)

    def goto(self):
        """Make the robot move to a specific x y"""
        self.twist = Twist()
        self.target = Pose2D(self.values["x"], self.values["y"], 0)
        rate = rospy.Rate(10)
        self.monitor.say("Commanded to goto")
        while not rospy.is_shutdown():
            self.distance = calc_distance(self.odom_pose, self.target)
            abs_required_turn = abs(normalize_angle(self.required_turn_angle))
            if  self.distance > 0.05:
                sig_turn = sigmoid(abs_required_turn, 0.5, rg.values["target_ang"])
                self.twist.linear.x = 0 if abs_required_turn > 0.3 else sigmoid(self.distance, 0.5, rg.values["target_lin"])
                self.twist.angular.z = sig_turn if normalize_angle(self.required_turn_angle) > 0 else -sig_turn
                self.safe_publish_cmd_vel()
            else:
                self.twist.linear.x = self.twist.angular.z = 0
                self.safe_publish_cmd_vel()
                return
            rate.sleep()

    def stop(self):
        """Stop the robot by setting the cmd_vel to all zeros"""
        self.twist = Twist()
        self.distance = 0
        self.monitor.say("Commanded to stop")
        self.safe_publish_cmd_vel()

    def move(self):
        """Move the robot a certain distance at a certain speed"""
        self.twist = Twist()
        rate = rospy.Rate(10)
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
            if self.distance < 0.05:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.safe_publish_cmd_vel()
                return
            rate.sleep()

# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("robogym")
    wait_for_simulator()
    cp = CommandInterface({"log": 1, "max_ang": 0.75, "max_lin": 0.5, "target_lin": 1.0, "target_ang" : 0.75})
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
        except KeyboardInterrupt:
            rg.stop()
            print("Stopping Robot. Type Quit or Exit to leave")

