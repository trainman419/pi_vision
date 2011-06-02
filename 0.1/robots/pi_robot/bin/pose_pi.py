#!/usr/bin/env python

"""
    Relax all joints on Pi Robot.
    
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
from sensor_msgs.msg import JointState
from arbotix.srv import Relax
import xml.dom.minidom
from math import pi

class pose_pi():
    def __init__(self):
        rospy.init_node("pose_pi")
                        
        current_joint_state = JointState()
        current_joint_state = self.get_joints()
       
        for joint in current_joint_state.name:
           service_name = joint + '_relax'
           rospy.wait_for_service(service_name)
           try:
               relaxProxy = rospy.ServiceProxy(service_name, Relax)
               relaxProxy()
           except rospy.ServiceException, e:
               print "Service call failed: %s"%e
        
        while not rospy.is_shutdown(): 
            rospy.spin()

    def get_joints(self):
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.free_joints = {}
        self.joint_list = [] # for maintaining the original order of the joints
        self.dependent_joints = rospy.get_param("dependent_joints", {})
        
        # Find all non-fixed joints
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                name = child.getAttribute('name')

                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if name in self.dependent_joints:
                    continue
                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.free_joints[name] = joint
                self.joint_list.append(name)

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()

        # Add Free Joints
        for (name, joint) in self.free_joints.items():
            joint_state.name.append(str(name))
            joint_state.position.append(joint['value'])
            
        return joint_state

        
if __name__ == '__main__':
    try:
        pose_pi()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down left arm forward kinematics node node...")


