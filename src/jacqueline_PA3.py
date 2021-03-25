#!/usr/bin/env python

# I looked at turtlebot3_teleop_key for reference
# start code from https://github.com/campusrover/prrexamples/blob/master/src/pa_starters/dance_pa_starter.py
import rospy
import sys
import math
import tf
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from math import pi, sqrt, atan2
import sys, select, os

# fill in scan callback
def scan_cb(msg):
    global ranges
    ranges = np.array(msg.ranges)

# update state according to key stroke
def key_cb(msg):
    global state; global queuing; global last_key_press_time; global state_list; global duration_list
    # we are using upper case letters
    state = msg.data.upper()
    if state == "Q" and not queuing and len(state_list) == 0:
        queuing = True
    elif state != "Q" and queuing:
        if state == "H" or state == "O":
            pass
        elif state == "L" or state == "R" or state == "F" or state == "B" or state == "S" or state == "Z":
            state_list.append(state)
            if len(state_list) > 1:
                duration_list.append((rospy.Time.now() - last_key_press_time).secs)
    elif state == "Q" and queuing:
        state = "Q_END"
        queuing = False
        duration_list.append((rospy.Time.now() - last_key_press_time).secs)
    elif state == "Q" and not queuing and len(state_list) > 0:
        state = "Q_EXEC"
    print(state)
    # print out the current state and time since last key press
    print_state()
    #update time
    last_key_press_time = rospy.Time.now()

# odom is also not necessary but very useful
def odom_cb(msg):
    global pose
    pose = msg.pose
    global x
    x = pose.pose.position.x
    global y
    y = pose.pose.position.y
    global yaw
    orientation_list = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

# print the state of the robot
def print_state():
    print("---")
    print("STATE: " + state)
    print(state_dict[state])

    if state == "Q_EXEC":
        print(state_list)
        print(duration_list)

    # calculate time since last key stroke
    time_since = rospy.Time.now() - last_key_press_time
    print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))

def transform_vec():
    if state == "H" or state == "L" or state == "R" or state == "F" or state == "B":
        return [1, 1]
    elif state == "S":
        nsec = (rospy.Time.now() - last_key_press_time).secs % 8
        if nsec >= 0 and nsec < 4:
            return [1, 1]
        else:
            return [0.5, 1]
    elif state == "Z":
        nsec = (rospy.Time.now() - last_key_press_time).secs % 8
        if nsec >= 0 and nsec < 2:
            return [1, 0]
        elif nsec >= 2 and nsec < 4:
            return [0, -1]
        elif nsec >= 4 and nsec < 6:
            return [1, 0]
        else:
            return [0, 1]
    else:
        return [1, 1]

# function to calculate which angle the robot needs to return 
def calc_turn_to():
    if x0 > x and y0 < y:
        turn_to = 3*pi/2 + atan2(abs(x0 - x), abs(y0 - y))
    elif x0 > x and y0 > y:
        turn_to = pi/2 - atan2(abs(x0 - x), abs(y0 - y))
    elif x0 < x and y0 > y:
        turn_to = pi/2 + atan2(abs(x0 - x), abs(y0 - y))
    elif x0 < x and y0 < y:
        turn_to = 3*pi/2 - atan2(abs(x0 - x), abs(y0 - y))
    elif x0 == x and y0 > y:
        turn_to = pi/2
    elif x0 == x and y0 < y:
        turn_to = 3*pi/2
    elif y0 == y and x0 < x:
        turn_to = pi
    elif y0 == y and x0 > x:
        turn_to = 0
    return turn_to

# function to wander around avoiding obstacles
def avoid_obs_move(ranges):
    if min(ranges[0:30]) >= threshold_1 and min(ranges[330:360]) >= threshold_1:
        twist.linear.x = forward_speed
        twist.angular.z = 0
    else:
        twist.linear.x = 0
        if min(ranges[0:50]) < min(ranges[310:360]):
            # if further obstacle on the right
            twist.angular.z = -abs_rotate
        else:
            # if further obstacle on the left
            twist.angular.z = abs_rotate
    cmd_vel_pub.publish(twist)

# function for the robot to go back to original point
def going_back(yaw):
    # calculate how far it is from the original point
    distance = sqrt(abs(x-x0)*abs(x-x0) + abs(y-y0)*abs(y-y0))
    turn_to = calc_turn_to()
    # making turn
    if yaw < 0 :
        yaw = yaw + 2*pi
    target = abs(yaw - turn_to)
    if turn_to > yaw:
        rotate_speed = abs_rotate
    else:
        rotate_speed = -abs_rotate
    # going straight forward
    twist.angular.z = rotate_speed
    twist.linear.x = 0
    tics = int(target / abs_rotate * r)
    for i in range(tics):
        cmd_vel_pub.publish(twist)
        rate.sleep()
    cmd_vel_pub.publish(Twist())
    tics = int(distance / forward_speed * r)
    for i in range(tics):
        avoid_obs_move(ranges)
        rate.sleep()
    cmd_vel_pub.publish(Twist())
    if distance <= 0.25:
        global back
        back = True
    
def exec_queue():
    global state_list; global duration_list
    
# this function takes care of "L", "R", "F", "B", "S", "Z", "H"
def exec_state(state):
    # generate vectors
    velocity_vector = VEL_DICT[state]
    transform_vector = transform_vec()

    # publish cmd_vel from here 
    twist = Twist()

    # get vector components
    linear_component = velocity_vector[0]
    angular_component = velocity_vector[1]
    linear_transform = transform_vector[0]
    angular_transform = transform_vector[1]

    # calculate velocities
    twist.linear.x = LINEAR_DEFAULT * linear_component * linear_transform
    twist.angular.z = ANGULAR_DEFAULT * angular_component * angular_transform

    # publish
    cmd_vel_pub.publish(twist)

# init node
rospy.init_node('dancer')

# subscribers/publishers
scan_sub = rospy.Subscriber('scan', LaserScan, scan_cb)

# RUN rosrun prrexamples key_publisher.py to get /keys
key_sub = rospy.Subscriber('keys', String, key_cb)
odom_sub = rospy.Subscriber('odom', Odometry, odom_cb)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# start in state halted and grab the current time
state = "H"
temp_state = "H"
last_key_press_time = rospy.Time.now()

# set rate
r = 10
rate = rospy.Rate(r)

# initialize scan
ranges = []

# max speeds and default speeds for burger model
MAX_LINEAR_SPEED = 0.22
MAX_ANGULAR_SPEED = 2.84
LINEAR_DEFAULT = 0.2
ANGULAR_DEFAULT = pi / 4

# default vectors
velocity_vector = [0, 0] # [linear_component, angular_component]
transform_vector = [1, 1] # [linear_transform, angular_transform]

# init pose
pose = None

twist = Twist()

queuing = False

# set absolute speed for robot
forward_speed = LINEAR_DEFAULT
abs_rotate = ANGULAR_DEFAULT
# set obstacle threshold distance
threshold_1 = 0.2

# velocity dictionary for velocity_vector = [linear_component, angular_component]
VEL_DICT = {
    "H": [0, 0],
    "L": [0, 1],
    "R": [0, -1],
    "F": [1, 0],
    "B": [-1, 0],
    "S": [1, -1],
    "Z": [1, 1],
    "O": [1, 1],
    "Q": [0, 0],
    "Q_END": [0,0]
}

# state dictionary for printing
state_dict = {
    "H": ">>>>>>>>>>> ON HALT <<<<<<<<<<<",
    "L": ">>>>>>>> ROTATING LEFT <<<<<<<<",
    "R": ">>>>>>>> OTATING RIGHT <<<<<<<<",
    "F": ">>>>>>>> GOING FORWARD <<<<<<<<",
    "B": ">>>>>>>> GOING BACKWARD <<<<<<<<",
    "S": ">>>>>>>> DANCING SPIRAL <<<<<<<<",
    "Z": ">>>>>>>> DANCING ZIGZAG <<<<<<<<",
    "O": ">>>>> RETURNING BIRTH POINT <<<<<",
    "Q": ">>>>>>>> QUEUING STARTS <<<<<<<<",
    "Q_END": ">>>>>>>> QUEUING ENDS <<<<<<<<",
    "Q_EXEC": ">>>>>>>>> EXEC QUEUE <<<<<<<<<"
}

# queue key stroke, store state and duration in two lists
state_list = []
duration_list = []

# safe proof loops
while state == "H": continue
while ranges == []: continue
while pose == None: continue

# initialize position info
start_pose = pose
yaw = 0
x = pose.pose.position.x
y = pose.pose.position.y
x0 = start_pose.pose.position.x
y0 = start_pose.pose.position.y

# whether the robot is back at where it is spawned
back = False

# Wait for published topics, exit on ^c
while not rospy.is_shutdown():
    if (min(ranges[0:30]) < 0.2 or min(ranges[330:360]) < 0.2) and (state == "F" or state == "S" or state == "Z" or state == "Q" or state == "O"):
        state = "H"

    # state "O": going back to origin point
    if state == "O":
        while not back:
            going_back(yaw)
        back = False

    # state "Q_EXEC": execute stored command queue
    if state == "Q_EXEC":
        # go back to original point
        while not back:
            going_back(yaw)
        # will use the code outside of this spin to execute stored motion queue
        break
    else:
        # execute command based on state
        exec_state(state)

    # run at 10hz
    rate.sleep()

# execute the motion queue stored by using "Q" key strokes
if state == "Q_EXEC":
    print("ENJOY THE SHOW!!")
    # carry out the motion
    while len(state_list) > 0:
        # change state
        state = state_list.pop(0)
        last_key_press_time = rospy.Time.now()
        dur_secs = duration_list.pop(0)
        tics = dur_secs * r
        # execute motion
        for i in range(tics):
            exec_state(state)
            rate.sleep()
    # stop the robot
    cmd_vel_pub.publish(Twist())
    print("THANKS FOR WATCHING!!")