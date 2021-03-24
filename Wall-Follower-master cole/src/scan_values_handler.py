#!/usr/bin/env python

#This processes all of the scan values


import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, Float32
from prrexamples.msg import LidarFilter

#Process all the data from the LIDAR
def cb(msg):
    global state
    messy = msg.ranges
    minimum = msg.range_min
    inf = float('inf')
    #Wait Cole, why are you making an angle below the min 4? why not set it to zero like we've done before?? Well,
    #The way I'm treating lidar data is if its below the min, or above the max, we're going to ignore it. Therefore
    #how we choose to represent those digits is irrelavant so long as they won't be mistaken for real values. Since
    #I'm often going to be taking the smallest value from a range of readings in this code, it's just more convenient 
    #to set trash data above the max, rather than handling a bunch of cases where we want to take the min of a set 
    #of distances EXCEPT for values of 0.
    tidy = [angle if (angle > minimum) else 4 for angle in messy]
    tidy = [angle if (not (angle == inf)) else 4 for angle in messy]
    state = calc_state(tidy)
    pub_state.publish(state)
    filter = LidarFilter()
    filter.angles = tidy
    pub_pid.publish(filter)
    

#Uses the cleaned up scan data to determine state
def calc_state(ranges):
    zone_dict = map_zones(ranges)
    if (zone_dict.get("left") and not zone_dict.get("right")):
        return 1
    if (zone_dict.get("right") and not zone_dict.get("left")):
        return 2
    if (zone_dict.get("right") and zone_dict.get("front")):
        return 4
    if (zone_dict.get("left") and zone_dict.get("front")):
        return 3
    else:
        return 5
    

    
def map_zones(ranges):
    #We split the lidar data into sections of 36 degrees, mainly because thats how it was done in the paper by
    #Rafael and Santos. I imagine they did some testing to get to this value before publishing, so I trust
    #That these values effectively categorize the bot's surroundings.
    threshold = 3
    
    #When I tried to append the front angles inside the min function it resulted in a NoneType error, so I moved that process
    #up here.
    front = ranges[341:360]
    front.append(ranges[0:19])

    zones = {
    "front" : min(front) < threshold,
    "front_right" : min(ranges[19:56]) < threshold,
    "right" : min(ranges[56:93]) < threshold,
    "back_right" : min(ranges[93:130]) < threshold,
    "front_left" : min(ranges[304:341]) < threshold,
    "left" : min(ranges[267:304]) < threshold,
    "back_left" : min(ranges[230:267]) < threshold,
    }
    return zones


#Init node
rospy.init_node('scan_values_handler')

#Subscriber for LIDAR
sub = rospy.Subscriber('/scan', LaserScan, cb)

#Publishers
pub_state = rospy.Publisher('state', Int16, queue_size = 1)
pub_pid = rospy.Publisher('surrounds', LidarFilter, queue_size = 1)

#Rate object
rate = rospy.Rate(10)

#Keep the node running
while not rospy.is_shutdown():
    rate.sleep() 