#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

def set_navigation_goal():
    
    # Create a publisher to publish the navigation goal
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    
    # Create a PoseStamped message and set the goal pose
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()

    goal_msg.header.frame_id = 'fiducial_0'  # Replace with your root frame ID
    goal_msg.pose.position.x = 0.0
    goal_msg.pose.position.y = 0.0
    goal_msg.pose.orientation.w = 1.0
    
    # Publish the goal
    goal_pub.publish(goal_msg)
    
def check_navigation_status():    
    # Create a subscriber to listen to the navigation status
    status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, callback)
    
    rospy.spin()

def callback(status_msg):
    if len(status_msg.status_list) > 0:
        status = status_msg.status_list[0].status

        if status == 1:  # Goal is currently being processed
            rospy.loginfo("Robot is currently moving to the goal.")
        elif status == 3:  # Goal has been reached
            rospy.loginfo("Robot has reached the goal.")
        elif status == 4:  # Goal was aborted
            rospy.logwarn("Robot failed to reach the goal.")
        elif status == 5:  # Goal was rejected
            rospy.logwarn("Robot goal was rejected.")

if __name__ == '__main__':
    rospy.init_node('set_navigation_goal', anonymous=True)
    set_navigation_goal()
    check_navigation_status()