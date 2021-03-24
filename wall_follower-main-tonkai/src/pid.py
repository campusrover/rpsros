#!/usr/bin/env python

#This is a PID controller to determine twist values

import rospy
from state_definitions import *
from constants import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from wall_follower.msg import Lidar
from math import pi

def cb_state(msg):
    global state
    state = msg.data

#Callback for Lidar data
#Get min_distance from wall, angle for that distance and also the direction
#Perpendicular_angle is the angle perpendicular to the direction of the wall
def cb_lidar(msg):
    global min_distance
    global min_angle
    global dir
    global perpendicular_angle
    min_distance = msg.min_distance
    min_angle = msg.min_angle
    dir = msg.dir
    if dir == 1:
        perpendicular_angle = 270
    else:
        perpendicular_angle = 90

def constrain(val):
    return min(MAX_VEL, max(MIN_VEL, val))

#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('pid_twist', Twist, queue_size = 1)

#subscribe the state and lidarfilter
sub_state = rospy.Subscriber('state', Int16, cb_state)
sub_lidar = rospy.Subscriber('lidarfilter', Lidar, cb_lidar)

#Twist and rate object
t = Twist()
rate = rospy.Rate(10)
state = WAITING
min_distance = 1
min_angle = 90
#these two variable is for determine the turning direction
dir = 0
perpendicular_angle = 90
# use PID for distance
prev_error = 0
dis_integral = 0
# use P for angle

while not rospy.is_shutdown():
    if state == FOLLOWING:
        #calculate p component
        #if wall is at right side (dir == 1), then we should turn anti-clockwise
        #refering to the paper, p_component consists of two parts: distance correction + head angle correction
        #P_Orin is used to set the proportion of the angle
        p_component = dir * (WALL_DISTANCE - min_distance) + P_Orin * ((min_angle - perpendicular_angle)*pi/180)
        print("p_component", p_component)
        #calculate d component
        d_component = p_component - prev_error
        print("d_component", d_component)
        #calculate i component
        i_component = dis_integral + p_component
        print("i_component", i_component)
        if p_component == 0 and prev_error == 0:
            dis_integral = 0
        prev_error = p_component
        #Add them all together, multiplied by their respective tuning values, and multiply everything
        #by the angular velocity
        t.angular.z = constrain(ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component))
        print("pid z", t.angular.z)
        t.linear.x = LINEAR_SPEED
        #Publish the twist to the driver
        pub.publish(t)
    rate.sleep() 