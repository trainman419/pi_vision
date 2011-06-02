#!/usr/bin/env python

"""
    Helper functions for the Pi Robot Project
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

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
import xml.dom.minidom

def ros_quaternion_to_trans(q):
    q_out = [0]*4
    q_out[0] = q[3]
    q_out[1] = q[1]
    q_out[2] = q[2]
    q_out[3] = q[0]
    
    return q_out

def get_joints():
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        free_joints = {}
        joint_list = [] # for maintaining the original order of the joints
        dependent_joints = rospy.get_param("dependent_joints", {})
        
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

                if name in dependent_joints:
                    continue
                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                free_joints[name] = joint
                joint_list.append(name)

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()

        # Add Free Joints
        for (name, joint) in free_joints.items():
            joint_state.name.append(str(name))
            joint_state.position.append(joint['value'])
            joint_state.velocity.append(0)
            
        return joint_state