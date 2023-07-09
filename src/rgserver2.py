#!/usr/bin/env python3
"""This module puts up a command prompt and gives the user control by commands of the robot"""

from rgparser2 import Parser
import rospy
from bru_utils import (
    turn_to_target,
    wait_for_simulator,
    calc_distance,
    normalize_angle,
    sigmoid,
    offset_point,
)
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Define initial variables
INITIAL_VARIABLES = {
    "log": 1,  # 1 means verbose logging
    "max_ang": 0.75,  # maximum angular velcoicy (imposed by safe_publish)
    "max_lin": 0.5,  # maximum linear velocoty
    "target_ang": 0.75,  # angular velocity for normal speed
    "target_lin": 1.0,  # linear velocity for normal speed
    "r1": "[[0,0],[1,1],[0,0]]",  # A sample navigational route
    "arrival_delta": 0.05,  # How close counts as the same point for goto command
}


class RoboGym:
    """Contains all the actions for the rg command set."""

    def __init__(self):
        rospy.init_node("robogym")
        wait_for_simulator()
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.mon_pub = rospy.Publisher("/monitor", Mon, queue_size=1)
        self.target = Pose2D(1, 1, 0)
        self.odom_pose = Pose2D(0, 0, 0)
        self.required_turn_angle = 0
        self.twist = None
        self.distance = None
        self.values = None
        self.exit = False

    def odom_cb(self, msg: str):
        """ROS callback for Odometry Message"""
        oreuler = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([oreuler.x, oreuler.y, oreuler.z, oreuler.w])
        self.odom_pose = Pose2D(msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        self.required_turn_angle = turn_to_target(
            self.odom_pose.theta,
            self.odom_pose.x,
            self.odom_pose.y,
            self.target.x,
            self.target.y,
        )

    def command_table(self):
        """Simply returns the command table"""
        return {
            "exit": {
                "args": [""],
                "nargs": 0,
                "description": "Exit Robogym",
                "handler": self.exit,
                "usage": "Invalid command. Usage: exit",
            },
            "stop": {
                "args": [],
                "nargs": 0,
                "description": "Stop the robot",
                "handler": self.stop,
            },
            "goto": {
                "args": ["<x>", "<y>"],
                "description": "Go to given odometry coordinate",
                "handler": self.goto,
                "usage": "Invalid command. Usage: goto <x> <y>",
                "nargs": 2,
            },
        }

    def exit(self, args):
        """Execute "exit" command, to exit program"""
        print("Exiting now...")
        exit(0)

    def stop(self):
        """Execute the "stop" command, to Stop the robot by setting the cmd_vel to all zeros"""
        self.twist = Twist()
        self.distance = 0
        self.mon_pub.publish(Mon("state", "Stop immediately"))
        self.safe_publish_cmd_vel()

    def goto(self):
        """Make the robot move to a specific x y"""
        self.twist = Twist()
        self.target = Pose2D(self.values["x"], self.values["y"], 0)
        rate = rospy.Rate(10)
        self.mon_pub.publish(
            Mon("state", f"Goto odometry x,y {self.target.x} {self.target.y}")
        )
        while not rospy.is_shutdown():
            self.distance = calc_distance(self.odom_pose, self.target)
            abs_required_turn = abs(normalize_angle(self.required_turn_angle))
            if self.distance > rg.values["arrival_delta"]:
                sig_turn = sigmoid(abs_required_turn, 0.5, rg.values["target_ang"])
                self.twist.linear.x = (
                    0
                    if abs_required_turn > 0.3
                    else sigmoid(self.distance, 0.5, rg.values["target_lin"])
                )
                self.twist.angular.z = (
                    sig_turn
                    if normalize_angle(self.required_turn_angle) > 0
                    else -sig_turn
                )
                self.safe_publish_cmd_vel()
            else:
                self.twist.linear.x = self.twist.angular.z = 0
                self.safe_publish_cmd_vel()
                return
            rate.sleep()


# Main function.
if __name__ == "__main__":
    # Initialize the node and name it.
    rg = RoboGym()
    cp = Parser(INITIAL_VARIABLES, rg.command_table())
    while True:
        try:
            cp.cli()
        except KeyboardInterrupt:
            print("Stopping Robot. Type Quit or Exit to leave")
