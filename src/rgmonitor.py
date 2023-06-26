#!/usr/bin/env python3

"""Monitor the state of the robot"""
import rospy
from sound_play.libsoundplay import SoundClient
from rpsexamples.msg import Mon

class Monitor:
    """Eventually will be the Robot's brain stem keeping track that things
    are going ok. For now it just reports state changes"""
    def __init__(self):
        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0
        self.sub_monitor = rospy.Subscriber('monitor', Monitor, self.monitor_callback)
        self.sub_odom = rospy.Subscriber('', Monitor, self.monitor_callback)


    def monitor_callback(self, msg):
        """Callback when requests are made to monitor"""
        pass



    def say(self, message):
        """Text to speech the string over the speaker"""
        print(f"Status monitor: {message}")
        self.soundhandle.say(message, self.voice, self.volume)

    def run(self):
        pass

if __name__ == '__main__':
    mon = Monitor()
    mon.run()



