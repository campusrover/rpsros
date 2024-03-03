#!/usr/bin/env python
import rospy
import tf2_ros
import nav_msgs.srv
from nav_msgs.msg import Odometry  
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Quaternion, Twist  


def get_current_pose():
    curr_pose = PoseStamped()
    curr_pose.header.frame_id = "base_link"

    # Get current position from odom topic
    odom_data = rospy.wait_for_message("odom", Odometry)
    curr_pose.pose.position = odom_data.pose.pose.position

    # Get current orientation
    curr_pose.pose.orientation = quaternion_from_euler(0, 0, odom_data.twist.twist.angular.z)
    return curr_pose

# Create node
rospy.init_node('align_to_target') 

# Transform listener
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

# Publish velocity commands
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  

# Control loop
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    try:
        # Lookup transform
        trans = tfBuffer.lookup_transform("base_link", "fiducial_0", rospy.Time(0))
        
        # Set target pose from transform 
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position = trans.transform.translation
        target_pose.pose.orientation = trans.transform.rotation

        # Get plan from current pose  
        start = get_current_pose() 
        
        # Extract v,w from first trajectory point
        v = traj_resp.plan.velocities[0].linear.x 
        w = traj_resp.plan.velocities[0].angular.z
        
        # Send velocity command   
        # cmd_vel = Twist() 
        # cmd_vel.linear.x = v
        # cmd_vel.angular.z = w
        # pub.publish(cmd_vel)

    except Exception as e: 
        pass  
        
    rate.sleep()