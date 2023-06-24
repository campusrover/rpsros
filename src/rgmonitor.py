#!/usr/bin/env python3
# Based on say.py from sound_play

import sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Monitor:
    def __init__(self):
        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.volume = 1.0

    def say(self, message):
        print(f"Status monitor: {message}")
        self.soundhandle.say(message, self.voice, self.volume)

