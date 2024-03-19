import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def send_goal_to_move_base(target_frame):
    # Initialize the ROS node
    rospy.init_node('send_goal_to_move_base')

    # Create an action client for the move_base action server

    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    # Wait for the action server to start
    move_base_client.wait_for_server()

    # Create a MoveBaseGoal with the target frame as the goal frame
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = target_frame
    goal.target_pose.header.stamp = rospy.Time.now()

    # Set the position and orientation of the goal
    goal.target_pose.pose.position.x = 0    # Example x coordinate
    goal.target_pose.pose.position.y = 9    # Example y coordinate
    goal.target_pose.pose.orientation.w = 1.0  # Example orientation (quaternion)

    # Send the goal to the move_base action server
    move_base_client.send_goal(goal)

    # Wait for the action to complete (you can add a timeout if desired)
    move_base_client.wait_for_result()

    # Check the result of the action
    if move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Goal reached successfully!")
    else:
        rospy.loginfo("Failed to reach the goal.")

if __name__ == '__main__':
    try:
        # Specify the target frame you want to navigate to
        target_frame = 'map'

        # Send the goal to move_base
        send_goal_to_move_base(target_frame)
    except rospy.ROSInterruptException:
        pass