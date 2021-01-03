#!/usr/bin/env python
import cmd, sys
from turtle import *
import rospy

class Handlers:
    def __init__(self):
        self.node = False
        pass

    def get_node(self):
        if self.node:
            return True
        try:
            rospy.init_node('rc_node')
            self.node = True
        except:
            self.node = False
        return self.node

    def show(self, param):
        if self.check_roscore():
            return rospy.getparam("x")
        

class RosConsole(cmd.Cmd):
    def __init__(self):
        self.intro = 'Welcome to the ROS Console shell.   Type help or ? to list commands.'
        self.prompt = '> '
        self.handler = Handlers()
    
    def preloop(self):
        if self.handler.get_node():
            prompt = "> "
        else:
            prompt = "? "    

    def do_stop(self, arg):
        'Stop the robot immediately!'
        print (arg)
        print("Stopping.")
    
    def do_exit(self, arg):
        'Exit from rc'
        print('Thank you for using rc')
        return True

    def do_show(self, arg):
        'Show param'
        handler.show("x")
        print("param 1")

if __name__ == '__main__':
    RosConsole().cmdloop()
