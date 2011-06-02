#!/usr/bin/env python

"""
    Set Pi Robot's joints to specific angles
    
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
from math import radians, degrees

class head_track():
    def __init__(self):
        rospy.init_node("position_joints")
        
        self.cmd_joints = rospy.Publisher("/cmd_joints", JointState)
        
        rospy.Subscriber('joint_states', JointState, self.updateJointStatesCb)
        
        dynamixels = rospy.get_param("/arbotix/dynamixels", dict())
        self.joints = dynamixels.keys()
        self.n_joints = len(self.joints)
        
        self.joint_cmd = JointState()
        self.joint_cmd.name = self.joints
        self.joint_cmd.position = [0] * self.n_joints
        self.joint_cmd.velocity = [1] * self.n_joints
        self.moving = False
        self.joint_state = JointState()
        time.sleep(2)
        
        while not rospy.is_shutdown():
            velocity = radians(random.randrange(30, 45)) * self.n_joints
            self.joint_cmd.position[self.joints.index("head_pan_joint")] = 0.0
            self.joint_cmd.position[self.joints.index("head_tilt_joint")] = 0.0

            """ Publish the movement command. """
            self.cmd_joints.publish(self.joint_cmd)
            
            """ Wait for motoin to stop before going on to the next position. """
            rospy.sleep(3)
            
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
        head_track()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down head track node...")

