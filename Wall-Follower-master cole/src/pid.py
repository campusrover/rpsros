#!/usr/bin/env python

#This is a PID controller to determine twist values

import rospy
from state_definitions import *
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16, Float32
from prrexamples.msg import LidarFilter

#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6

#Multipliers used to tune the PID controller
#Proportional constant
P_CONSTANT = 0.5
P_ANG = 0.03
#Integral constant
I_CONSTANT = 0.05
#Derivative constant
D_CONSTANT = 0.5
global integral
integral = 0
global prev
prev = 0
global error
error = 0
global ang_error
ang_error = 0

#CALLBACKS FOR ANYTHING YOUR PID NEEDS TO SUBSCRIBE TO FROM scan_values_handler
def lidar(msg):
    global error
    global integral
    global prev
    global ang_error
    #previous error must be saved for the d component
    prev = error
    #We want to be 0.5 meter away from the wall. Error is the difference between this target and our actual distance.
    error = min(msg.angles) - 0.5
    ang_index = msg.angles.index(min(msg.angles))
    if ang_index < 180:
        ang_error = (msg.angles.index(min(msg.angles)) - 90)
    else:
        ang_error = (270 - msg.angles.index(min(msg.angles)))
    integral = integral + error

#Init node
rospy.init_node('pid')

#Create publisher for suggested twist objects
pub = rospy.Publisher('twist', Twist, queue_size = 1)

#SUBSCRIBERS FOR THINGS FROM scan_values_handler YOU MIGHT WANT
sub = rospy.Subscriber('/surrounds', LidarFilter, lidar)

#Twist and rate object
t = Twist()
rate = rospy.Rate(10)



while not rospy.is_shutdown():
    #calculate p component
    p_component = error + (ang_error * P_ANG)
    #calculate d component
    d_component = (error - prev)
    #calculate i component
    i_component = integral * 0.01
    #Add them all together, multiplied by their respective tuning values, and multiply everything
    #by the angular velocity
    t.angular.z = ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component)
    #if(error < 0.1 and error > 0):
    #    t.linear.x = 0.1 * LINEAR_SPEED
    #if(error < 0.25 and error > 0):
    #    t.linear.x = 0.25 * LINEAR_SPEED
    #if(error < 0.5 and error > 0):
    #    t.linear.x = 0.5 * LINEAR_SPEED
    #else:
    t.linear.x = LINEAR_SPEED
    #Publish the twist to the driver
    pub.publish(t)
    rate.sleep() 