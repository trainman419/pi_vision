#!/usr/bin/env python

"""
    talkback.py - Say back what is heard by the pocketsphinx recognizer.
"""

import roslib; roslib.load_manifest('pi_speech')
import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String

class TalkBack:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.soundhandle.stopAll()

        # Subscribe to the recognizer output
        rospy.Subscriber('recognizer/output', String, self.talkback)
        
        while not rospy.is_shutdown():
            rospy.sleep(1)
        
    def talkback(self, msg):
        rospy.loginfo(msg.data)
        self.soundhandle.say(msg.data)     

    def cleanup(self):
        self.soundhandle.stopAll
        rospy.loginfo("Shutting down talkback node...")

if __name__=="__main__":
    rospy.init_node('talkback')
    try:
        TalkBack()
    except:
        pass

