#!/usr/bin/env python
import cmd, sys
import os
import rospy
import socket
import tldextract
import platform    # For getting the operating system name
import subprocess  # For executing a shell command

class Handlers:
    def __init__(self):
        self.node = False
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

    def check_rosmaster(self):
        self.get_master_uri()
        try:
            host = socket.gethostbyname(self.master_domainname)
            s = socket.create_connection((host, 11311), 2)
            s.close()
            self.ros_master = True
        except:
            pass
        self.ros_master = False
        return self.ros_master

    def ping(self, host):
        """
        Returns True if host (str) responds to a ping request.
        Remember that a host may not respond to a ping (ICMP) request even if the host name is valid.
        """
        # Option for the number of packets as a function of
        param = '-n' if platform.system().lower()=='windows' else '-c'

        # Building the command. Ex: "ping -c 1 google.com"
        command = ['ping', param, '1', host]
        return subprocess.call(command) == 0
     
class RosConsole(cmd.Cmd):
    intro = 'Welcome to the ROS Console shell.   Type help or ? to list commands.'
    handlers = Handlers()
    prompt = "> "

    def do_wtf(self, arg):
        'Check network, roscore, etc.'
        self.handlers.get_master_uri()
        if self.handlers.ping(self.handlers.master_domainname):
            print("ping master uri OK")
        if self.handlers.check_rosmaster():
            print("roscore OK")
        if self.handlers.init_node():
            print("node created.")

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
    rc = RosConsole()
    rc.cmdloop()