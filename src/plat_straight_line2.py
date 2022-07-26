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
from basenode2 import BaseNode
import bru_utils as bu

class StraightLine(BaseNode):

    def __init__(self):
        super().__init__()
        self.initial_pose : Pose2D
        self.current_pose: Pose2D
        self.dist_trav: float
        self.odom_sub: rospy.Subscriber
        self.movement_pub: rospy.Publisher = None
        self.pose_pub: rospy.Publisher = None
        self.start: rospy.Time
        self.state: str

    def log(self):
        bu.info(f"curr:{self.current_pose.x:1.2},{self.current_pose.y:1.2}")

    def odom_cb(self, msg):
        self.current_pose.x, self.current_pose.y = msg.pose.pose.position.x, msg.pose.pose.position.y
        roll, pitch, self.current_pose.theta = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        if self.initial_pose is None:
            self.initial_pose = copy.copy(self.current_pose)
        self.dist = bu.calc_distance(self.current_pose, self.initial_pose)
        self.pose_pub.publish(self.current_pose)
    
    def initial_setup(self):
        super().initial_setup()
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.movement_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose", Pose2D, queue_size=1)
        self.rate = rospy.Rate(10)
        self.start = rospy.Time.now()
        self.dist = 0.0
        self.current_pose = Pose2D()
        self.initial_pose = None

    def drive_straight_line(self, arrived) ->  None:
        bu.info("straight driving")
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.rate.sleep()
        while not arrived(self.dist):
            self.log()
            self.movement_pub.publish(twist)
            self.rate.sleep()
        twist.linear.x = 0.0
        self.movement_pub.publish(twist)
        self.rate.sleep()

    def turn_by_radians(self, arrived) -> None:
        bu.info("turning")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        while not arrived(self.current_pose.theta) and not self.shutdown_requested:
            self.movement_pub.publish(twist)
            self.rate.sleep()
        self.movement_pub.publish(twist)
        self.rate.sleep()

    def stop(self) -> None:
        super().stop()
        twist = Twist()
        self.movement_pub.publish(twist)

    def loop(self):
        super().loop()
        for i in range(1):
            self.drive_straight_line(lambda d: d >= 0.5)
            target_theta = bu.invert_angle(self.current_pose.theta)
            self.turn_by_radians(lambda d: isclose(target_theta, self.current_pose.theta, abs_tol=0.2))
            self.drive_straight_line(lambda d: d <= 0.0)
        self.stop()


if __name__ == '__main__':
    rospy.init_node('plat_straight_line2')
    rn = StraightLine()
    rospy.on_shutdown(rn.shutdown_hook)
    rn.run()

