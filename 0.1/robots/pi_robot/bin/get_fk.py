#!/usr/bin/env python

"""
    Use forward kinematics to get the pose of the left finger tip given the
    current joint angles of the left arm. 
    
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
from kinematics_msgs.msg import KinematicSolverInfo
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionFK, GetPositionFKRequest
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Quaternion

import time

class get_fk():
    def __init__(self):
        rospy.init_node("pi_robot_get_fk")
        
        self.joint_state = JointState()
        
        self.rate = 1
        r = rospy.Rate(self.rate)
        
        q = quaternion_from_euler(0, 0, 0)
        
        #rospy.loginfo(q)                
        rospy.wait_for_service('left_arm_kinematics/get_fk')
        rospy.wait_for_service('left_arm_kinematics/get_fk_solver_info')

        get_fk_proxy = rospy.ServiceProxy('left_arm_kinematics/get_fk', GetPositionFK, persistent=True)
        get_fk_solver_info_proxy = rospy.ServiceProxy('left_arm_kinematics/get_fk_solver_info', GetKinematicSolverInfo)

        left_arm_solver_info = get_fk_solver_info_proxy()
        
#        self.left_arm_joints = list()
#        for joint in solver_info.kinematic_solver_info.joint_names:
#            self.left_arm_joints.append(joint)
#            rospy.loginfo("Adding joint " + str(joint))
            
        rospy.Subscriber('joint_states', JointState, self.getJointState)
        
        self.request = GetPositionFKRequest()
        self.request.robot_state.joint_state = JointState()
        self.request.robot_state.joint_state.header.frame_id = 'lower_torso_link'
        self.request.robot_state.joint_state.name = left_arm_solver_info.kinematic_solver_info.joint_names
        self.request.robot_state.joint_state.position = [0]*len(self.request.robot_state.joint_state.name)
        #self.request.robot_state.joint_state.position[0] = 1.0

        self.request.header.frame_id = "lower_torso_link"
        self.request.fk_link_names = list()
        self.request.fk_link_names.append("left_shoulder_pan_link")
        self.request.fk_link_names.append("left_shoulder_lift_link")
        self.request.fk_link_names.append("left_arm_roll_link")
        self.request.fk_link_names.append("left_elbow_link")
        self.request.fk_link_names.append("left_wrist_link")
        self.request.fk_link_names.append("left_hand_link")
        self.request.fk_link_names.append("left_finger_link")
                
        while not rospy.is_shutdown(): 
            try:
                response = get_fk_proxy(self.request)
                hand_link = response.pose_stamped[0]
                q = list()
                q.append(hand_link.pose.orientation.x)
                q.append(hand_link.pose.orientation.y)
                q.append(hand_link.pose.orientation.z)
                q.append(hand_link.pose.orientation.w)
                rpy = euler_from_quaternion(q)
                rospy.loginfo(rpy)
                rospy.loginfo(response.pose_stamped) 
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
        rospy.loginfo("Shutting down left arm forward kinematics node...")


