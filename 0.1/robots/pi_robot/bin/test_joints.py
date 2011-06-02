#!/usr/bin/env python

"""
    Test all of Pi Robot's joints
    
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
from sensor_msgs.msg import JointState
import pi_tracker_lib as PTL
from math import radians

class test_joints():
    def __init__(self):
        rospy.init_node("test_joints")
        
        self.cmd_joints = rospy.Publisher("/cmd_joints", JointState)
        
        #rospy.Subscriber('joint_states', JointState, self.updateJointStatesCb)
        #rospy.Subscriber('extra_joint_states', ExtraJointState, self.updateExtraJointStatesCb)  
        
        self.joint_cmd = JointState()
        self.joint_cmd = PTL.get_joints()
        self.joints = self.joint_cmd.name
        self.n_joints = len(self.joints)
        
        self.joint_cmd.position = [0] * self.n_joints
        self.joint_cmd.velocity = [1] * self.n_joints
        self.moving = False
        
        self.joint_state = JointState()
        rospy.sleep(2)
        
        pub_rate = 2
        
        self.limited_joints = ('left_shoulder_pan_joint', 'left_elbow_joint', 'right_shoulder_pan_joint', 'right_elbow_joint')        

        while not rospy.is_shutdown():
            velocity = radians(random.randrange(30, 70))
            self.joint_cmd.velocity = [radians(50)] * self.n_joints
            self.joint_cmd.position = [radians(random.randrange(-45, 45)) for x in range(self.n_joints)]
            
            try:
                for joint in self.limited_joints:
                    self.joint_cmd.position[self.joint_cmd.name.index(joint)] = radians(random.randrange(-5, 5))
            except:
                pass
            
            #self.joint_cmd.position = [radians(random.randrange(-100, 100))] * self.n_joints
            """ Scale the movement speeds so that they take the same amount of time. """
#            max_delta = 0
#            for i in range(len(self.joints)):
#                delta = abs(self.joint_cmd.position[i] - self.joint_state.position[i])
#                if delta > max_delta:
#                    max_delta = delta
#            for i in range(len(self.joints)):
#                self.joint_cmd.velocity[i] = velocity * abs(self.joint_cmd.position[i] - self.joint_state.position[i]) / (max_delta + 0.01)
#                self.joint_cmd.velocity[i] = min(2.0, max(0.1, self.joint_cmd.velocity[i])) # Remember that for the AX-12 servo, speed = 0 means "as fast as you can"]
#            
            """ Publish the movement command. """
            self.cmd_joints.publish(self.joint_cmd)
            rospy.sleep(pub_rate)
                        
            """ Wait for motoin to stop before going on to the next position. """
            #while self.moving and not rospy.is_shutdown():
                #rospy.sleep(0.2)

            
    def updateJointStatesCb(self, msg):
        #rospy.loginfo(str(msg.position[2]))
        self.joint_state = msg
        
    def updateExtraJointStatesCb(self, msg):
        for is_moving in msg.moving:
            if is_moving:
                self.moving = True
                return
        self.moving = False
                    
if __name__ == '__main__':
    try:
        test_joints()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down test joints node...")

