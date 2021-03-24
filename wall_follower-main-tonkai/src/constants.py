INF = 10 # sign for infinit
WALL_DISTANCE = 1
# limitation on angular speed
MIN_VEL = -1
MAX_VEL = 1
# env: 0-front 1-front_left 2-left 3-Back_Left 4-Back_rear_left 5-rear 6-back_rear_right 7-back_right 8-right 9-front_right
FRONT = 0
FRONT_LEFT = 1
LEFT = 2
BACK_LEFT = 3
BACK_REAR_LEFT = 4
REAR = 5
BACK_REAR_RIGHT = 6
BACK_RIGHT = 7
RIGHT = 8
FRONT_RIGHT = 9

#Linear speed of the robot
LINEAR_SPEED = 0.3
#Angular speed of the robot
ANGULAR_SPEED = 3.1415926/6

#Multipliers used to tune the PID controller
#Proportional constant
P_CONSTANT = 15
#Integral constant
I_CONSTANT = 0
#Derivative constant
D_CONSTANT = 5
#Orientaion portion
P_Orin = 0.2