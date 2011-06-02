#!/usr/bin/env python

"""
    Use inverse kinematics to get joint positions of the left arm necessary to move
    the left finger tip to a given pose. 
    
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

import sys
import roslib; roslib.load_manifest('pi_demo')
import rospy
from sensor_msgs.msg import JointState
from kinematics_msgs.msg import KinematicSolverInfo, PositionIKRequest
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
import xml.dom.minidom
from math import pi

import time

class get_ik():
    def __init__(self):
        rospy.init_node("pi_robot_get_ik")
                
        self.joint_state_pub = rospy.Publisher("joint_states", JointState)
        self.joint_state_update = JointState()
        self.joint_state_update = self.get_joints()
        self.joint_state_update.position = [0]*len(self.joint_state_update.name)
        
        #rospy.Subscriber('joint_states', JointState, self.getJointState)
        
        #self.joint_state_update = self.init_joint_state
        
        self.rate = 10
        r = rospy.Rate(self.rate)
        
        self.left_right = 'left'
        self.ref_frame = 'left_shoulder_link'
        self.target_frame = 'left_hand_link'
                        
        rospy.wait_for_service(self.left_right + '_arm_kinematics/get_ik')
        rospy.wait_for_service(self.left_right + '_arm_kinematics/get_ik_solver_info')

        get_ik_proxy = rospy.ServiceProxy(self.left_right + '_arm_kinematics/get_ik', GetPositionIK, persistent=True)
        get_ik_solver_info_proxy = rospy.ServiceProxy(self.left_right + '_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)

        arm_solver_info = get_ik_solver_info_proxy()
                    
        request = GetPositionIKRequest()
        request.timeout = rospy.Duration(5.0)
        request.ik_request.pose_stamped.header.frame_id = 'left_shoulder_link'
        request.ik_request.ik_link_name = 'left_hand_link'
        request.ik_request.pose_stamped.pose.position.x =  0.240380409633
        request.ik_request.pose_stamped.pose.position.y =  0.0459488642754
        request.ik_request.pose_stamped.pose.position.z =  0.0831832121611
        
        request.ik_request.pose_stamped.pose.orientation.x = -0.0634909451402
        request.ik_request.pose_stamped.pose.orientation.y = -0.664584733057
        request.ik_request.pose_stamped.pose.orientation.z = -0.493509516449
        request.ik_request.pose_stamped.pose.orientation.w = 0.557444517102

        request.ik_request.ik_seed_state.joint_state.name = arm_solver_info.kinematic_solver_info.joint_names
        rospy.loginfo(request.ik_request.ik_seed_state.joint_state.name)
        request.ik_request.ik_seed_state.joint_state.position = [0]*len(request.ik_request.ik_seed_state.joint_state.name)
        
        #rospy.loginfo(request)
        
        self.response = get_ik_proxy(request)
        
        rospy.loginfo(self.response)
                
        while not rospy.is_shutdown(): 
#            try:
#                self.response = get_ik_proxy(request)
#                rospy.loginfo(self.response) 
#                for joint in request.ik_request.ik_seed_state.joint_state.name:
#                    try:
#                        self.joint_state_update.position[self.joint_state_update.name.index(joint)] = self.response.solution.joint_state.position[self.response.solution.joint_state.name.index(joint)]
#                        # Reseed the solver with the current joint state.
#                        #request.ik_request.ik_seed_state.joint_state.position[request.ik_request.ik_seed_state.joint_state.name.index(joint)] = self.joint_state_update.position[self.joint_state_update.name.index(joint)]
#                    except ValueError:
#                        rospy.loginfo("Value Error: " + joint)
#                self.joint_state_update.header.stamp = rospy.Time.now()
#                self.joint_state_pub.publish(self.joint_state_update)
#                      
#                #request.ik_request.pose_stamped.pose.position.x -= 0.001
#                #request.ik_request.pose_stamped.pose.position.y -= 0.001
#                #request.ik_request.pose_stamped.pose.position.z -= 0.001
#
#            except rospy.ServiceException, e:
#                print "Service call failed: %s"%e
            
            r.sleep()
            
    def getJointState(self, msg):
        for joint in self.request.robot_state.joint_state.name:
            self.init_joint_state.position[self.joint_state.name.index(joint)] = msg.position[msg.name.index(joint)]
                   

    def get_joints(self):
        """ This function is take from the joint_state_publisher package written by David Lu!!
            See http://www.ros.org/wiki/joint_state_publisher
        """
        description = rospy.get_param("robot_description")
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        free_joints = {}
        joint_list = [] # for maintaining the original order of the joints
        dependent_joints = rospy.get_param("dependent_joints", {})
        
        # Find all non-fixed joints.
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
    
        # Add Free Joints.
        for (name, joint) in free_joints.items():
            joint_state.name.append(str(name))
            joint_state.position.append(joint['value'])
            joint_state.velocity.append(0)
            
        return joint_state
   
if __name__ == '__main__':
    try:
        get_ik()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down arm inverse kinematics node...")


