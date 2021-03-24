#!/usr/bin/env python

#This processes all of the scan values


import rospy
from state_definitions import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from wall_follower.msg import Lidar
from constants import *

#Process all the data from the LIDAR
#Representation of the environment: assume the head is at degree 0
#Use the region model in the paper, each region size is 36 degrees
def cb(msg):
    range_lst = filter(msg)
    #publish lidarfilter for pid control
    lidar = Lidar()
    lidar.min_distance = INF
    for i in range(len(range_lst)):
        if range_lst[i] < lidar.min_distance:
            lidar.min_distance = range_lst[i]
            lidar.min_angle = i
    if lidar.min_angle > 180:
        lidar.dir = 1 # wall is at right
    else:
        lidar.dir = -1
    pub_lidar.publish(lidar)
    #calculate the min_distance in each region to generate the environment array
    env = calculate_min_region(range_lst)
    print(env)
    s = determine_state(env)
    #Determine state
    pub_state.publish(s)
    
#This function is a filter modify the raw scan data
#INF is a constant which can be defined in constants.py
#INF means that if range >= INF, then the robot will ignore obstacle in that direction
#In a narrow environment, we may set a smaller INF 
def filter(msg):
    ranges = list(msg.ranges)
    for i in range(len(msg.ranges)):
        if ranges[i] < msg.range_min:
            ranges[i] = msg.range_min
        elif ranges[i] > INF:
            ranges[i] = INF
    return ranges

# each region spans 36 degrees
def calculate_min_region(range_lst):
    res = []
    # we represent the env by set env[0] = min(region_front)
    range_rotate = []
    j = -18 # start of the front_region
    count = 0
    min = None
    for i in range(len(range_lst)):
        count += 1
        curr = range_lst[(j + 360) % 360]
        j += 1
        if min == None or curr < min:
            min = curr
        if count % 36 == 0:
            res.append(min)
            count = 0
            min = None
    return res

# Determine the state from the surrounding environment detected
def determine_state(env):
    if env[FRONT] < INF and env[FRONT_LEFT] < INF and env[FRONT_RIGHT] < INF and env[LEFT] < INF and env[RIGHT] < INF:
        s = DEAD_END
    elif (env[FRONT_LEFT] < INF and env[LEFT] < INF) or (env[RIGHT] < INF and env[FRONT_RIGHT] < INF):
        s = FOLLOWING
    elif env[BACK_REAR_RIGHT] < INF or env[BACK_RIGHT] < INF:
        s = RIGHT_TURN
    else:
        s = LEFT_TURN
    return s


#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('scan', LaserScan, cb)

#Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
pub_lidar = rospy.Publisher('lidarfilter', Lidar, queue_size = 1)


#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 