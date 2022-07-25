 #!/usr/bin/env python3

import rospy
from scipy.spatial import distance
from geometry_msgs.msg import Pose2D
import numpy as np
from math import pi, sqrt, atan2, isclose

# Stand alone functions

# Simple trig
# angles are in radians, -pi < angle < pi. 0 is at 3 o'clock

def calc_distance(pa: Pose2D, pb: Pose2D) -> float:
    return distance.euclidean([pa.x, pa.y], [pb.x, pb.y])

def invalid_pose(pose: Pose2D) -> bool:
    return pose.x == 0 and pose.y == 0 and pose.theta == 0

def invert_angle(angle: float) -> float:
    return (angle + pi) % (pi)

def offset_point(point: Pose2D, distance: float) -> Pose2D:
    pass

# Logging and   debugging  functions
def info(msg: str):
    rospy.loginfo(msg)

# Very simpl(istic) and easy to understand PID implementation
class PID():
    def __init__(self, min_val, max_val, kp, ki, kd):
        self.min_val = min_val
        self.max_val = max_val 
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        self.derivative = error - self.prev_error
        if setpoint == 0 and self.prev_error == 0: self.integral = 0
        pid = (self.kp * error + self.ki * self.integral + self.kd * self.derivative)
        self.prev_error = error
        # r = np.clip(pid, self.min_val, self.max_val)
        # print(f"** PID {pid:1.2} {error:1.2} {r:1.2}")
        return np.clip(pid, self.min_val, self.max_val)
