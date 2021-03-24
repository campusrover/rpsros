HOW TO RUN: please use $roslaunch wall_follower wall_follower.launch

This will launch all three of the nodes correctly if they are marked as executable.

The robot should be initially placed near a wall.

##Implementation Notes:
Reference paper: Wall-following algorithm for reactive autonomous mobile robot with laser scanner sensor

###Constants:
constants.py
state_definitions.py

###Modules:
scan_values_handler.py: 

    subscriber of `scan`, use the raw scan data to analysis the environment. We split the range of the scan data into 10 regions and just
    pick the shortest distance in each region to denote the property for that region. To recognize the environment, we basically check at which 
    region does the robot detect walls(distance < INF).

    Thus, the determine_state function takes in the environment array, returns the state recognized.
    Also, the min_distance to the wall, angle and direction is published for pid controling
driver.py: 

    subscriber of `scan_values_handler` and `pid`, send cmd_vel to the robot based on the states
pid.py: 

    subsciber of `scan_values_handler`, formular to get the tuned angular velocity

    angular_vel = constrain(ANGULAR_SPEED * (P_CONSTANT * p_component + D_CONSTANT * d_component + I_CONSTANT * i_component))
    Note that p_component consists of two parts: distance correction + (head angle correction) * P_ORI
    distance correction is calculated by the min_distance, while head angle correction is from the angle.
    The purpose for using both angle and distance is that we can make the robot facing the same direction as the wall to get a smoother trajectory


    gain tuning:
    result: P_CONSTANT=15 I_CONSTANT=0 D_CONSTANT=5 P_ORI=0.2

###Challenges

1. following wall: implemented by the state FOLLOWING
2. right turn: implemented by the state RIGHT_TURN
3. dead end: turn around implemented by the state DEAD_END
4. narrower: not implemented yet
5. round wall: there is no assumption for the wall shape, so it works naturally in the state FOLLOWING

