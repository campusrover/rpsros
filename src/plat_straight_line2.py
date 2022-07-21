#!/usr/bin/env python3

from typing import Callable
import rospy
import tf2_ros
import signal
import copy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sqrt, atan2, isclose
import basenode2
import bru_utils as bu

class StraightLine(basenode2.BaseNode.BaseNode):
    initial_pose: Pose2D
    current_pose: Pose2D
    dist: float
    odom_sub: rospy.Subscriber
    movement_pub: rospy.Publisher
    pose_pub: rospy.Publisher
    start: rospy.Time
    state: str

    def odom_cb(self, msg):
        self.current_pose.x, self.current_pose.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        roll, pitch, self.current_pose.theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        if self.initial_pose is None:
            self.initial_pose = copy.copy(self.current_pose)
            print(f"initial:{self.initial_pose.x:1.2},{self.initial_pose.y:1.2}")
        self.dist = bu.calc_distance(self.current_pose, self.initial_pose)
        self.pose_pub.publish(self.current_pose)
    
    def initial_setup(self):
        rospy.loginfo("init")
        rospy.init_node("line")
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
        self.rate = rospy.Rate(10)
        self.start = rospy.Time.now()
        self.dist = 0.0
        self.current_pose = Pose2D()
        self.initial_pose = None
        rospy.loginfo("move forward")

    def drive_straight_line(self, arrived) ->  None:
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.movement_pub.publish(twist)
        self.rate.sleep()
        while not arrived(self.dist):
            self.rate.sleep()
        twist.linear.x = 0.0
        self.movement_pub.publish(twist)
        self.rate.sleep()

    def turn_180(self, target_theta: float) -> None:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.movement_pub.publish(twist)
        self.rate.sleep()
        while not isclose(target_theta, self.current_pose.theta, abs_tol=0.2):
            self.rate.sleep()
        self.movement_pub.publish(twist)
        self.rate.sleep()

    def stop(self) -> None:
        twist = Twist()
        self.movement_pub.publish(twist)

    def loop(self):
        for i in range(1):
            bu.info("out")
            self.drive_straight_line(lambda d: d >= 0.5)
            bu.info("turn")
            self.turn_180()
            bu.info = "return"
            self.drive_straight_line(lambda d: d <= 0.0)
        self.stop()
