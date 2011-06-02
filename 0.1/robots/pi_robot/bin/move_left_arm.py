#!/usr/bin/env python

"""
    Use inverse kinematics update the joint positions of the left arm necessary to move
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

import roslib; roslib.load_manifest('pi_robot')
import rospy
from sensor_msgs.msg import JointState
from kinematics_msgs.msg import KinematicSolverInfo, PositionIKRequest
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
from tf.transformations import quaternion_from_euler
import xml.dom.minidom
from math import pi
import time

class move_arm():
    def __init__(self):
        rospy.init_node("move_arm")
                
        armPub = rospy.Publisher("/cmd_joints", JointState)
        
        current_joint_state = JointState()
        current_joint_state = self.get_joints()
        
        self.rate = 0.3
        r = rospy.Rate(self.rate)
                        
        rospy.wait_for_service('left_arm_kinematics/get_ik')
        rospy.wait_for_service('left_arm_kinematics/get_ik_solver_info')
        
        get_ik_proxy = rospy.ServiceProxy('left_arm_kinematics/get_ik', GetPositionIK, persistent=True)
        get_ik_solver_info_proxy = rospy.ServiceProxy('left_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)

        left_arm_solver_info = get_ik_solver_info_proxy()
                    
        request = GetPositionIKRequest()
        request.timeout = rospy.Duration(5.0)
        request.ik_request.pose_stamped.header.frame_id = "left_shoulder_link";
        request.ik_request.ik_link_name = "left_finger_link";
        request.ik_request.pose_stamped.pose.position.x =  0
        request.ik_request.pose_stamped.pose.position.y = 0.078
        request.ik_request.pose_stamped.pose.position.z = 0
        
#        request.ik_request.pose_stamped.pose.orientation.x = 0
#        request.ik_request.pose_stamped.pose.orientation.y = 0
#        request.ik_request.pose_stamped.pose.orientation.z = 0
#        request.ik_request.pose_stamped.pose.orientation.w = 1
        
        q = quaternion_from_euler(0, 0, 0)

        request.ik_request.pose_stamped.pose.orientation.x = q[0]
        request.ik_request.pose_stamped.pose.orientation.y = q[1]
        request.ik_request.pose_stamped.pose.orientation.z = q[2]
        request.ik_request.pose_stamped.pose.orientation.w = q[3]

        request.ik_request.ik_seed_state.joint_state.name = left_arm_solver_info.kinematic_solver_info.joint_names
        request.ik_request.ik_seed_state.joint_state.position = [0]*len(request.ik_request.ik_seed_state.joint_state.name )
        
        arm_joints = left_arm_solver_info.kinematic_solver_info.joint_names
        
        arm_cmd = JointState()
        arm_cmd.name = arm_joints
        arm_cmd.position = request.ik_request.ik_seed_state.joint_state.position
        for joint in arm_joints:
            arm_cmd.position[arm_cmd.name.index(joint)] = current_joint_state.position[current_joint_state.name.index(joint)]
        arm_cmd.velocity = [1]*len(arm_joints)
        
        for joint in arm_joints:
            arm_cmd.position[arm_cmd.name.index(joint)] = 0
       
        arm_cmd.header.stamp = rospy.Time.now()
        arm_cmd.header.frame_id = 'left_shoulder_link'
        armPub.publish(arm_cmd)
        
        r.sleep()
        
        while not rospy.is_shutdown(): 
            try:
                response = get_ik_proxy(request)
                rospy.loginfo(response) 
                for joint in arm_joints:
                    arm_cmd.position[arm_cmd.name.index(joint)] = response.solution.joint_state.position[response.solution.joint_state.name.index(joint)]

                arm_cmd.header.stamp = rospy.Time.now()
                arm_cmd.header.frame_id = 'left_shoulder_link'
                armPub.publish(arm_cmd)

                request.ik_request.pose_stamped.pose.position.x = 0
                request.ik_request.pose_stamped.pose.position.y = 0.078
                request.ik_request.pose_stamped.pose.position.z = 0
                
                q = quaternion_from_euler(0, 0, 0)
        
                request.ik_request.pose_stamped.pose.orientation.x = q[0]
                request.ik_request.pose_stamped.pose.orientation.y = q[1]
                request.ik_request.pose_stamped.pose.orientation.z = q[2]
                request.ik_request.pose_stamped.pose.orientation.w = q[3]
                
#                request.ik_request.pose_stamped.pose.orientation.x = 0
#                request.ik_request.pose_stamped.pose.orientation.y = 0
#                request.ik_request.pose_stamped.pose.orientation.z = 0
#                request.ik_request.pose_stamped.pose.orientation.w = 1

            except rospy.ServiceException, e:
                print "Service call failed: %s"%e
            
            #rospy.loginfo("HELLO!!")  
            r.sleep()
            

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
        move_arm()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down left arm forward kinematics node node...")


