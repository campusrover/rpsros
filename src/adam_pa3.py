#!/usr/bin/env python

import rospy
from os import system
import sys
import math
import tf
import time
import random
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


  

class Controller:


    '''
    Initializes a Controller object
    '''
    def __init__(self, max_vel=0.3):

        rospy.init_node('dancer')


        self.max_vel = max_vel

        # subscribers/publishers
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)
        self.key_sub = rospy.Subscriber('keys', String, self.key_cb)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.move = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # start in state halted and grab the current time
        self.state = " "
        self.last_key_press_time = rospy.Time.now()

        # set rate
        self.rate = rospy.Rate(10)

        # sets the starting position and time to measure speed
        self.start_pos = [0, 0]
        self.start_time = rospy.Time.now().to_sec()


        # a start time for special movements that require time delays such as
        # zig_zag() and go_crazy()
        self.special_movement_time = rospy.Time.now().to_sec()

        # creates dictionary that associates each movement function with the key press required to 
        # execute it as its dictionary key
        self.movement_dict = {
                            'e': self.exit,
                            ' ':self.halt,
                            'a':self.rotate_left,
                            'd':self.rotate_right,
                            'w':self.move_forward,
                            's':self.move_backward,
                            'j':self.rotate_left_halting,
                            'l':self.rotate_right_halting,
                            'i':self.move_forward_halting,
                            'k':self.move_backward_halting,
                            'f':self.spiral,
                            'z':self.zig_zag,
                            'c':self.go_crazy,
                            'r':self.move_to_clearing
                            }


        # Twist instance
        self.twist = Twist()


    '''
    Scan callback creates list of ranges from lidar as an instance for this class
    '''
    def scan_cb(self, msg):
        self.ranges = list(msg.ranges)

        
    '''
    Key callback creates a string as the last keypress as an instance, 
    a time object as an instance indicating the last key press, and a 
    special movement last key press time for movements that require time
    delays such as zig_zag() or go_crazy()
    '''
    def key_cb(self, msg):
        self.state = msg.data
        self.last_key_press_time = rospy.Time.now()
        self.special_movement_time = rospy.Time.now().to_sec()


    '''
    Creates instances for the robot's x and y position and quaternion angles
    '''
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.xq = msg.pose.pose.orientation.x
        self.yq = msg.pose.pose.orientation.y
        self.zq = msg.pose.pose.orientation.z
        self.wq = msg.pose.pose.orientation.w


    '''
    Returns the robot's current rotation in radians about the 
    z axis
    '''
    def get_rotation(self):

        t3 = 2.0 * (self.wq * self.zq + self.xq * self.yq)
        t4 = 1.0 - 2.0 * (self.yq * self.yq + self.zq * self.zq)
        rot = math.atan2(t3, t4)
        return rot


    '''
    Returns the distance between two points 
    '''
    @staticmethod
    def dist(start, end):
        return math.sqrt((start[0] - end[0])**2 + (start[1] - end[1])**2)


    '''
    Returns the difference between two numbers
    '''
    @staticmethod
    def diff(a, b):
        return abs(a-b)
    

    '''
    Returns the robot's current speed by measuring the distance
    traveled over a specified amount of time, in this case the 
    amount of time would be the rate of the controller
    '''
    def curr_speed(self):
        end_pos = [self.x, self.y]
        end_time = rospy.Time.now().to_sec()
        # calculating the distance traveled between start_time and end_time
        distance_traveled = self.dist(self.start_pos, end_pos)
        # calculating change in time
        self.time_delta = end_time - self.start_time
        # avoids divide by 0 error if time_delta happens to be 0
        if self.time_delta == 0:
            speed = 0
        else:  
            speed = distance_traveled / self.time_delta
        self.start_pos = end_pos
        self.start_time = end_time
        return speed


    '''
    Prints current information about the robot, including the current state/keypress,
    the current time since the last keypress in seconds, the current speed of the robot
    in meters per second, and the current position of the robot in meters
    '''
    def print_state(self):
        system('clear')
        print("---")
        print("STATE: " + self.state)

        # calculate time since last key stroke
        time_since = rospy.Time.now() - self.last_key_press_time
        print("SECS SINCE LAST KEY PRESS: " + str(time_since.secs))
        print("CURRENT SPEED: " + str(round(self.curr_speed(), 3)) + " M/S")
        print("CURRENT POSITION: (" + str(round(self.x, 2)) + ", " + str(round(self.y,2)) + ")")
        

    '''
    Sleeps the robot for 0.1 seconds
    '''
    def sleep(self):
        self.rate.sleep()


    '''
    Returns the shortest distance between the robot and
    any obstacle near it as given by the lidar
    '''
    def collision(self):
        self.collision_angle = self.ranges.index(min(self.ranges))
        return min(self.ranges)


    '''
    Returns the shortest distance between the robot
    and any obstacle in front of it as given by the lidar
    '''
    def front_collision(self):
        return min(self.ranges[0:20] + self.ranges[340:360])


    '''
    Returns the shortest distance between the robot and
    any obstacle in back of it as given by the lidar
    '''
    def back_collision(self):
        return min(self.ranges[160:200])


    '''
    Exits the program
    '''
    def exit(self):
        self.halt()
        sys.exit()


    '''
    Sets the robot's angular z and linear x velocities to 0
    '''
    def halt(self):
        self.twist.linear.x = 0
        self.twist.angular.z = 0
        self.move.publish(self.twist)
    

    '''
    Sets the robot's angular z velocity to a positive number
    '''
    def rotate_left(self):
        self.twist.angular.z = math.pi/4
        self.move.publish(self.twist)


    '''
    First halts all motion in the robot, then sets the robot's
    angular z velocity to a positive number
    '''
    def rotate_left_halting(self):
        self.halt()
        self.twist.angular.z = math.pi/4
        self.move.publish(self.twist)


    '''
    Sets the robot's angular z velocity to a negative number
    '''
    def rotate_right(self):
        self.twist.angular.z = -math.pi/4
        self.move.publish(self.twist)


    '''
    First halts all motion in the robot, then sets the robot's
    angular z velocity to a negative number
    '''
    def rotate_right_halting(self):
        self.halt()
        self.twist.angular.z = -math.pi/4
        self.move.publish(self.twist)


    '''
    Defines an asymptotic function that can only reach up to the
    specified maximum velocity and approaches close to 0 when the
    specified difference approaches 0.

    Desmos link for both linear and rotational breaking curves:
    https://www.desmos.com/calculator/nj1sgvhtxb
    linear : green
    rotational : red
    '''
    @staticmethod
    def correct_vel_linear(diff, max_vel):
        
        return -((max_vel*5)/(2**(20*diff + 2.36))) + max_vel


    '''
    Sets the robot's linear x velocity to a positive number
    '''
    def move_forward(self):
        # corrects the linear velocity of the robot so that the robot will break in 
        # order to not get closer than 0.2 meters to a wall in front of it. This 
        # technique is used for all methods that set the robot's linear x velocity
        # to a non-zero number
        stopping_distance = self.front_collision()
        self.twist.linear.x = self.correct_vel_linear(stopping_distance, self.max_vel)
        self.move.publish(self.twist)


    '''
    First halts all motion in the robot, then sets the robot's
    linear x velocity to a positive number
    '''
    def move_forward_halting(self):
        self.halt()
        stopping_distance = self.front_collision()
        self.twist.linear.x = self.correct_vel_linear(stopping_distance, self.max_vel)
        self.move.publish(self.twist)


    '''
    Sets the robot's linear x velocity to a negative number
    '''
    def move_backward(self):
        stopping_distance = self.back_collision()
        self.twist.linear.x = self.correct_vel_linear(stopping_distance, -self.max_vel)
        self.move.publish(self.twist)


    '''
    First halts all motion in the robot, then sets the robot's
    linear x velocity to a negative number
    '''
    def move_backward_halting(self):
        self.halt()
        stopping_distance = self.back_collision()
        self.twist.linear.x = self.correct_vel_linear(stopping_distance, -self.max_vel)
        self.move.publish(self.twist)


    '''
    Moves the robot in a forward spiral for 8 seconds, and then moves the robot
    in a backwards spiral for another 8 seconds. Repeats these motions every 16 seconds
    '''
    def spiral(self):
        self.halt()
        time_since = self.diff(rospy.Time.now().to_sec(), self.special_movement_time)
        if time_since <= 8:
            max_vel = (-(self.max_vel*5)/2**(time_since + 2.3)) + self.max_vel
            ang_vel = math.pi/4
            stopping_distance = self.front_collision() - 0.2
        else:
            max_vel = ((self.max_vel*5)/2**(time_since + 10.3)) - self.max_vel
            ang_vel = -math.pi/4
            stopping_distance = self.back_collision() - 0.2
        if time_since >= 16:
            self.special_movement_time = rospy.Time.now().to_sec()
        self.twist.linear.x = self.correct_vel_linear(stopping_distance, max_vel)
        self.twist.angular.z = ang_vel
        self.move.publish(self.twist)


    '''
    Moves the robot in a zig-zag motion. The angular velocity of the 
    robot changes sign depending on the time since the last change, or the
    time since the start of the movement indicated by the special_movement_time
    instance. The linear x velocity stays constant at 0.3 meters per second
    '''
    def zig_zag(self):
        time_since = rospy.Time.now().to_sec() - self.special_movement_time
        print("Time since: " + str(time_since))
        if time_since <= 2:
            vel = -math.pi/3
        else:
            vel = math.pi/3
        if time_since >= 4:
            self.special_movement_time = rospy.Time.now().to_sec()
        print("Angular velocity: " + str(vel))
        stopping_distance = self.front_collision() - 0.2
        self.twist.linear.x = self.correct_vel_linear(stopping_distance, self.max_vel)
        self.twist.angular.z = vel
        self.move.publish(self.twist)


    '''
    Rotates the robot at a high angular velocity in different directions at
    randomly assigned times. The time that the robot rotates in a specific direction
    is dependent on a random integer between 2 and 5 and the instance special_movement_time 
    such that the robot will rotate in one direction for a random time between 2 seconds 
    and 8 seconds
    '''
    def go_crazy(self):
        rand_duration = random.randint(2,5)
        time_since = rospy.Time.now().to_sec() - self.special_movement_time
        if time_since <= rand_duration:
            vel = -2*math.pi
        else:
            vel = 2*math.pi
        if time_since >= rand_duration * 2:
            self.special_movement_time = rospy.Time.now().to_sec()
        self.twist.angular.z = vel
        self.move.publish(self.twist)


    '''
    Returns the direction that the robot will have to turn to reach
    a target angle so that the robot turns the minimum distance possible
    '''
    @staticmethod
    def get_direction(curr, target):
        return np.sign((curr - target) * (abs(curr-target) - math.pi))


    '''
    Defines an asymptotic function that can only reach up to the
    specified maximum velocity and approaches close to 0 when the
    specified difference approaches 0.

    Desmos link for both linear and rotational breaking curves:
    https://www.desmos.com/calculator/nj1sgvhtxb
    linear : green
    rotational : red
    '''    
    @staticmethod
    def correct_vel_rotation(diff, max_vel):
        return -((max_vel*5)/(2**(2*diff + 2.4))) + max_vel

    
    '''
    Calculates the angle that the robot has to rotate to in order
    to point towards the origin point (0,0). Derived from the 
    formula to calculate the angle between two vectors.
    '''
    def calculate_angle_to_origin(self):
        angle = abs(math.acos(self.x/(math.sqrt(self.x**2 + self.y**2)+1)))
        if self.y > 0.2:
            return angle - math.pi
        else:
            return math.pi - angle


    '''
    Rotates the robot until its current direction is the 
    pointing in the direction of the specified angle
    '''
    def rotate_towards_angle(self, angle):
        curr = self.get_rotation()
        diff = self.diff(curr,angle)
        direction = self.get_direction(curr, angle)
        while (diff > 0.005):
            curr = self.get_rotation()
            diff = self.diff(curr, angle)
            vel = self.correct_vel_rotation(diff, math.pi/3) * direction
            self.twist.angular.z = vel
            self.move.publish(self.twist)
        self.twist.angular.z = 0
        self.move.publish(self.twist)


    '''
    In the case that the robot runs too close to a wall, the robot will
    halt. Since the intance state is always telling the robot to halt if it
    is close to a wall, there is no good way to direct the robot away from a
    wall if it runs into on. This function ignores the current state and directs
    the robot to move towards the origin until it is at least 0.5 meters away
    from nearest wall.
    '''
    def move_to_clearing(self):
        system('clear')
        self.halt()
        print('Moving to clearing...')
        curr_ranges = self.ranges
        angle = self.calculate_angle_to_origin()
        self.rotate_towards_angle(angle)
        stopping_distance = 0.5 - self.collision()
        while stopping_distance > 0 and self.state != 'e':
            self.twist.linear.x = self.correct_vel_linear(stopping_distance, self.max_vel)
            self.move.publish(self.twist)
            stopping_distance = 0.5 - self.collision()
        self.state = ' '
        system('clear')
        self.print_state


    '''
    This is the function that uses user input from the key publisher to
    move the robot based on the input. The last input from the user/the instance
    state is used as a key to index into the dictionary containing all of the
    movement functions for the robot. This method will run the function with the key
    as the last key press in the dictionary instance movement_dict. If the robot is
    within 0.2 meters of a wall, the robot halts and the state is set to ' '. If the
    user inputs a command that is not assigned to a movement function, the controller
    will simply ignore it
    '''         
    def main_controller(self):
        if (self.collision() <= 0.2) and (self.state != 'e' and self.state != 'r'):
            self.state = ' '
            self.halt()
        elif self.movement_dict.has_key(self.state): 
            self.movement_dict[self.state.lower()]()
        else:
            pass


max_vel = input("Enter the maximum velocity of\nthe robot in meters per second\n(Warning, maximum velocities over\n0.3 m/s are not known to be stable)\n")


# initializes the Controller object
ct = Controller(max_vel = max_vel)


# sleeps the program for half of a second so that the robot has time
# to publish odometry and lidar topics
time.sleep(0.5)


'''
Main control loop continues while the rospy node is not shutdown. Controls the
rate at which the controller publishes (10 hz), prints the state of the robot, 
and controls the robot based on user input using the main_controller() method
'''
while not rospy.is_shutdown():
   ct.rate.sleep()
   ct.print_state()
   ct.main_controller() 
