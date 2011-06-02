#!/usr/bin/env python

"""
    Move the head to track a target given by (x, y) coordinates
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import roslib; roslib.load_manifest('pi_robot')
import rospy
import time
import random
#from arbotix.srv import *
from sensor_msgs.msg import JointState
from math import radians

class head_track():
    def __init__(self):
        rospy.init_node("test_head")
        
        self.cmd_joints = rospy.Publisher("/cmd_joints", JointState)
        
        rospy.Subscriber('joint_states', JointState, self.updateJointStatesCb)
        
        self.head_cmd = JointState()
        self.joints = ["head_pan_joint", "head_tilt_joint"]
        self.head_cmd.name = self.joints
        self.head_cmd.position = [0, 0]
        self.head_cmd.velocity = [1, 1]
        self.moving = False
        self.joint_state = JointState()
        time.sleep(2)
        
        while not rospy.is_shutdown():
            velocity = radians(random.randrange(30, 50))
            #velocity = radians(60)
            self.head_cmd.position = [radians(random.randrange(-90, 90)), radians(random.randrange(-60, 60))]
            #self.head_cmd.position = [radians(random.randrange(-60, 60))]*2
            
            """ Sync the pan and tilt movements so they take the same amount of time. """
            max_delta = 0
            for i in range(len(self.joints)):
                delta = abs(self.head_cmd.position[i] - self.joint_state.position[i])
                if delta > max_delta:
                    max_delta = delta
            for i in range(len(self.joints)):
                self.head_cmd.velocity[i] = velocity * abs(self.head_cmd.position[i] - self.joint_state.position[i]) / (max_delta + 0.01)
                #self.head_cmd.velocity[i] = max(0.1, self.head_cmd.velocity[i]) # Remember that for the AX-12 servo, speed = 0 means "as fast as you can"]
            
            """ Publish the movement command. """
            self.cmd_joints.publish(self.head_cmd)
            
            """ Wait for motoin to stop before going on to the next position. """
            while self.moving and not rospy.is_shutdown():
                rospy.sleep(0.2)
                
            rospy.sleep(3)

    def updateJointStatesCb(self, msg):
        #rospy.loginfo(str(msg.moving))
        self.joint_state = msg           
                    
if __name__ == '__main__':
    try:
        head_track()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down head track node...")


