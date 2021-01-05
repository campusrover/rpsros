#!/usr/bin/env python
import cmd, sys
import os
import rospy
import socket
import tldextract
import platform    # For getting the operating system name
import subprocess  # For executing a shell command
from geometry_msgs.msg import Twist


class Handlers:
    def __init__(self):
        self.node = False
        self.cmd_vel = None
        pass

    def init_node(self):
        if self.node:
            return True
        try:
            rospy.init_node('rc_node')
            self.node = True
        except:
            self.node = False
        return self.node

    def get_master_uri(self):
        self.master_uri = os.getenv('ROS_MASTER_URI')
        self.master_domainname = tldextract.extract(self.master_uri).domain
        return self.master_uri != "" and self.master_uri != ""

    def check_rosmaster(self):
        self.get_master_uri()
        self.ros_master = False
        try:
            host = socket.gethostbyname(self.master_domainname)
            s = socket.create_connection((host, 11311), 2)
            s.close()
            self.ros_master = True
        except:
            pass
        return self.ros_master

    def ping_host(self, host):
        """
        Returns True if host (str) responds to a ping request.
        Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
        """
        # Option for the number of packets as a function of
        param = '-n' if platform.system().lower()=='windows' else '-c'

        # Building the command. Ex: "ping -c 1 -t 1 -q google.com"
        command = ['ping', param, '1', "-t 1 -q", host]
        self.ping = subprocess.call(command) == 0
        return self.ping
    
    def check_cmd_vel(self):
        if not (self.node):
            return False
        if not(self.cmd_vel):
            self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def cmd_vel(speed):
        if c.cmd_vel_pub == None:
            return False
        t = Twist()
        t.linear.x = 0.1
        self.cmd_vel_pub.publish(ts)

    def forward(self):
        if self.check_cmd_vel():
            self.cmd_vel(0.2)
            print("OK")
     
class RosConsole(cmd.Cmd):
    intro = 'Welcome to the ROS Console shell.   Type help or ? to list commands.'
    h = Handlers()
    prompt = "> "

    def do_connect(self, arg):
        'Check network, roscore, etc.'
        if self.h.get_master_uri():
            if self.h.ping_host(self.h.master_domainname):
                if self.h.check_rosmaster():
                    if self.h.init_node():
                        print("OK")
                    else:
                        print("failed to create a node")
                else:
                    print("failed to connect to roscore")
            else:
                print("failed to ping master uri")
        else:
            print("failed to locate environment variables")

    def do_stop(self, arg):
        'Stop the robot immediately!'
        print (arg)
        print("Stopping.")
    
    def do_exit(self, arg):
        'Exit from rc'
        print('Thank you for using rc')
        return True

    def do_quit(self, arg):
        return self.do_exit(arg)

    def do_show(self, arg):
        'Show param'
        h.show("x")
        print("param 1")
    
    def do_forward(self, arg):
        'Move robot forward'
        self.h.forward(*self.parse(arg))

    def parse(self,arg):
        'Convert a series of zero or more numbers to an argument tuple'
        return tuple(map(int, arg.split()))

if __name__ == '__main__':
    rc = RosConsole()
    rc.cmdloop()