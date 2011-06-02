#!/usr/bin/env python

"""
    Use forward kinematics to get the pose of the left or right finger tip given the
    current joint angles of the arm. 
    
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

import roslib; roslib.load_manifest('pi_demo')
import rospy
from sensor_msgs.msg import JointState
from kinematics_msgs.msg import KinematicSolverInfo
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionFK, GetPositionFKRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion

import time

class get_fk():
    def __init__(self):
        rospy.init_node("pi_robot_get_fk")
        
        self.left_right  = 'left'
        self.ref_frame = 'left_shoulder_link'
        
        self.joint_state = JointState()
        
        self.rate = 1
        r = rospy.Rate(self.rate)
        
        q = quaternion_from_euler(0, 0, 0)
        
        #rospy.loginfo(q)                
        rospy.wait_for_service(self.left_right + '_arm_kinematics/get_fk')
        rospy.wait_for_service(self.left_right + '_arm_kinematics/get_fk_solver_info')

        get_fk_proxy = rospy.ServiceProxy(self.left_right + '_arm_kinematics/get_fk', GetPositionFK, persistent=True)
        get_fk_solver_info_proxy = rospy.ServiceProxy(self.left_right + '_arm_kinematics/get_fk_solver_info', GetKinematicSolverInfo)

        arm_solver_info = get_fk_solver_info_proxy()
        
#        self.left_right _arm_joints = list()
#        for joint in solver_info.kinematic_solver_info.joint_names:
#            self.left_right _arm_joints.append(joint)
#            rospy.loginfo("Adding joint " + str(joint))
            
        rospy.Subscriber('joint_states', JointState, self.getJointState)
        
        self.request = GetPositionFKRequest()
        self.request.robot_state.joint_state = JointState()
        self.request.robot_state.joint_state.header.frame_id = self.ref_frame
        self.request.robot_state.joint_state.name = arm_solver_info.kinematic_solver_info.joint_names
        self.request.robot_state.joint_state.position = [0]*len(self.request.robot_state.joint_state.name)
        #self.request.robot_state.joint_state.position[0] = 1.0

        self.request.header.frame_id = self.ref_frame
        self.request.fk_link_names = list()
        self.request.fk_link_names.append(self.left_right + "_shoulder_pan_link")
        self.request.fk_link_names.append(self.left_right + "_shoulder_lift_link")
        self.request.fk_link_names.append(self.left_right + "_arm_roll_link")
        self.request.fk_link_names.append(self.left_right + "_elbow_link")
        self.request.fk_link_names.append(self.left_right + "_forearm_link")
        self.request.fk_link_names.append(self.left_right + "_wrist_link")
        self.request.fk_link_names.append(self.left_right + "_hand_link")
        self.request.fk_link_names.append(self.left_right + "_finger_link")
                
        while not rospy.is_shutdown(): 
            try:
                response = get_fk_proxy(self.request)
                target_link = response.pose_stamped[6]
                q = list()
                q.append(target_link.pose.orientation.x)
                q.append(target_link.pose.orientation.y)
                q.append(target_link.pose.orientation.z)
                q.append(target_link.pose.orientation.w)
                rpy = euler_from_quaternion(q)
                #rospy.loginfo(rpy)
                #rospy.loginfo(response.pose_stamped)
                rospy.loginfo(target_link)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
            r.sleep()
            
    def getJointState(self, msg):
        for joint in self.request.robot_state.joint_state.name:
            self.request.robot_state.joint_state.position[self.request.robot_state.joint_state.name.index(joint)] = msg.position[msg.name.index(joint)]
                   
if __name__ == '__main__':
    try:
        get_fk()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down arm forward kinematics node...")


