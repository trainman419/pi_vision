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
sys.path.append("/home/patrick/Eclipse/ros/pi-robot-ros-stack/prl/src")

import roslib; roslib.load_manifest('pi_robot')
import rospy
from sensor_msgs.msg import JointState
from kinematics_msgs.msg import KinematicSolverInfo, PositionIKRequest
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
import xml.dom.minidom
from math import pi
from prl import get_joints

import time

class get_ik():
    def __init__(self):
        rospy.init_node("pi_robot_get_ik")
                
        self.joint_state_pub = rospy.Publisher("joint_states", JointState)
        self.joint_state_update = JointState()
        self.joint_state_update = get_joints()
        
        self.rate = 10
        r = rospy.Rate(self.rate)
                        
        rospy.wait_for_service('left_arm_kinematics/get_ik')
        rospy.wait_for_service('left_arm_kinematics/get_ik_solver_info')

        get_ik_proxy = rospy.ServiceProxy('left_arm_kinematics/get_ik', GetPositionIK, persistent=True)
        get_ik_solver_info_proxy = rospy.ServiceProxy('left_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)

        left_arm_solver_info = get_ik_solver_info_proxy()
                    
        request = GetPositionIKRequest()
        request.timeout = rospy.Duration(5.0)
        request.ik_request.pose_stamped.header.frame_id = "torso_link";
        request.ik_request.ik_link_name = "left_hand_link";
        request.ik_request.pose_stamped.pose.position.x =  0.266796872165
        request.ik_request.pose_stamped.pose.position.y = 0.0
        request.ik_request.pose_stamped.pose.position.z = 0.292845877564
        
#        request.ik_request.pose_stamped.pose.orientation.x = 0.159293225427
#        request.ik_request.pose_stamped.pose.orientation.y = 0.898180025306
#        request.ik_request.pose_stamped.pose.orientation.z = 0.0735797897789
#        request.ik_request.pose_stamped.pose.orientation.w = -0.403093444514
        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = -1.57
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.707

        request.ik_request.ik_seed_state.joint_state.name = left_arm_solver_info.kinematic_solver_info.joint_names
        request.ik_request.ik_seed_state.joint_state.position = [0]*len(request.ik_request.ik_seed_state.joint_state.name )
                
        while not rospy.is_shutdown(): 
            try:
                self.response = get_ik_proxy(request)
                rospy.loginfo(self.response) 
                for joint in request.ik_request.ik_seed_state.joint_state.name:
                    self.joint_state_update.position[self.joint_state_update.name.index(joint)] = self.response.solution.joint_state.position[self.response.solution.joint_state.name.index(joint)]
                self.joint_state_update.header.stamp = rospy.Time.now()
                self.joint_state_pub.publish(self.joint_state_update)
                request.ik_request.pose_stamped.pose.position.x -= 0.0005
                request.ik_request.pose_stamped.pose.position.y = 0.0
                request.ik_request.pose_stamped.pose.position.z -= 0.0005

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
            r.sleep()
   
if __name__ == '__main__':
    try:
        get_ik()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down left arm inverse kinematics node...")


