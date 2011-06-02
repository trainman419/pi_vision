#!/usr/bin/env python

"""
    Control the joints of a robot using a skeleton tracker such as the
    OpenNI tracker package in junction with a Kinect RGB-D camera.
    
    Based on Taylor Veltrop's C++ work (http://www.ros.org/wiki/veltrop-ros-pkg)
    
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

import roslib; roslib.load_manifest('demo_tracker')
import rospy
import demo_tracker_lib as PTL
from sensor_msgs.msg import JointState
from demo_tracker.msg import Skeleton
from demo_tracker.srv import *
from std_msgs.msg import Float64
import PyKDL as KDL
from ax12_controller_core.srv import SetSpeed, TorqueEnable, SetTorqueLimit
from math import acos, asin, atan, atan2, pi

class TrackerJointController():
    def __init__(self):
        rospy.init_node('tracker_joint_controller')
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Joint Controller Node...")
        
        self.rate = rospy.get_param('~joint_controller_rate', 5)
        rate = rospy.Rate(self.rate)
        
        self.controller_namespace = rospy.get_param('controller_namespace', '/ax12_controller')
        self.skel_to_joint_map = rospy.get_param("~skel_to_joint_map", dict())
        self.use_real_robot = rospy.get_param('~use_real_robot', False)
        self.mirror = rospy.get_param('~mirror', False)
        self.skeleton_lost = True

        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.3)
        self.tracker_commands = rospy.get_param('~tracker_commands', ['STOP', 'TELEOP_JOINTS'])
        
        # Dictionaries for various ax12_controller services
        self.set_speed = dict()
        self.torque_enable = dict()
        self.set_torque_limit = dict()
        
        dynamixels = rospy.get_param(self.controller_namespace+'/dynamixels', dict())
        for name in sorted(dynamixels):
            rospy.wait_for_service(self.controller_namespace+'/'+name+'_controller/set_speed')
            rospy.wait_for_service(self.controller_namespace+'/'+name+'_controller/torque_enable')  
            rospy.wait_for_service(self.controller_namespace+'/'+name+'_controller/set_torque_limit')    
            self.set_speed[name] = rospy.ServiceProxy(self.controller_namespace+'/'+name+'_controller/set_speed', SetSpeed)
            self.torque_enable[name] = rospy.ServiceProxy(self.controller_namespace+'/'+name+'_controller/torque_enable', TorqueEnable)
            self.set_torque_limit[name] = rospy.ServiceProxy(self.controller_namespace+'/'+name+'_controller/set_torque_limit', SetTorqueLimit)
            self.set_torque_limit[name](0.2)
            self.set_speed[name](self.default_joint_speed)
            self.torque_enable[name](True)
     
        self.set_torque_limit['head_pan'](0.5)
        self.set_torque_limit['head_tilt'](0.5)
        self.set_torque_limit['left_shoulder_lift'](1.0)
        self.set_torque_limit['right_shoulder_lift'](1.0)
         
        self.HALF_PI = pi / 2.0

        # Subscribe to the skeleton topic.
        rospy.Subscriber('skeleton', Skeleton, self.skeleton_handler)
        
        # Store the current skeleton configuration in a local dictionary.
        self.skeleton = dict()
        self.skeleton['confidence'] = dict()
        self.skeleton['position'] = dict()
        self.skeleton['orientation'] = dict()
         
        # Set up the tracker command service for this node.
        self.set_command = rospy.Service('~set_command', SetCommand, self.set_command_callback)

        # We only use the joint_state publisher if we are not using a real robot (so we can see them in RViz)
        if not self.use_real_robot:
            self.joint_state_pub = rospy.Publisher('/joint_states', JointState)      
        else:
            # The joints are controlled by publishing joint positions on the ax12 controller topics.
            self.head_tilt_pub = rospy.Publisher('/ax12_controller/head_tilt_controller/command', Float64)
            self.head_pan_pub = rospy.Publisher('/ax12_controller/head_pan_controller/command', Float64)
            self.left_shoulder_pan_pub = rospy.Publisher('/ax12_controller/left_shoulder_pan_controller/command', Float64)
            self.left_shoulder_lift_pub = rospy.Publisher('/ax12_controller/left_shoulder_lift_controller/command', Float64)
            self.left_arm_roll_pub = rospy.Publisher('/ax12_controller/left_arm_roll_controller/command', Float64)
            self.left_elbow_pub = rospy.Publisher('/ax12_controller/left_elbow_controller/command', Float64)
            self.left_forearm_pub = rospy.Publisher('/ax12_controller/left_forearm_controller/command', Float64)
            self.left_wrist_pub = rospy.Publisher('/ax12_controller/left_wrist_controller/command', Float64)
            self.right_shoulder_pan_pub = rospy.Publisher('/ax12_controller/right_shoulder_pan_controller/command', Float64)
            self.right_shoulder_lift_pub = rospy.Publisher('/ax12_controller/right_shoulder_lift_controller/command', Float64)
            self.right_arm_roll_pub = rospy.Publisher('/ax12_controller/right_arm_roll_controller/command', Float64)
            self.right_elbow_pub = rospy.Publisher('/ax12_controller/right_elbow_controller/command', Float64)
            self.right_forearm_pub = rospy.Publisher('/ax12_controller/right_forearm_controller/command', Float64)
            self.right_wrist_pub = rospy.Publisher('/ax12_controller/right_wrist_controller/command', Float64)
            self.torso_pub = rospy.Publisher('/ax12_controller/torso_controller', Float64)
        
        # The get_joints command parses a URDF description of the robot to get all the non-fixed joints.
        self.cmd_joints = JointState()
        self.cmd_joints = PTL.get_joints()
        
        # Store the last joint command so we can stop and hold a given posture.
        self.last_cmd_joints = JointState()
        self.last_cmd_joints = PTL.get_joints()
        
        # Initialize the robot in the TELEOP_JOINTTS state.
        self.tracker_command = "TELEOP_JOINTS"
        
        if self.mirror:
            left = "right"
            right = "left"
            reflect = -1
        else:
            left = "left"
            right = "right"
            reflect = 1
        
        while not rospy.is_shutdown():    
            # If no robot is attached, just simulate movement of the joints.
            if not self.use_real_robot:
                self.teleop_joints()
                self.joint_state = self.cmd_joints
                self.joint_state_pub.publish(self.joint_state)
            else:      
                # Execute the behavior appropriate for the current command.
#                if self.skeleton_lost:
#                    rospy.loginfo('SKELETON LOST!')
#                    self.tracker_command = 'STOP'
#                else:
#                    self.tracker_command = 'TELEOP_JOINTS'
#                    try:
#                        if self.skeleton['confidence']['torso'] != 1:
#                            rospy.loginfo('TORSO LOST')
#                            self.tracker_command = 'STOP'
#                        else:
#                            self.tracker_command = 'TELEOP_JOINTS'
#                    except:
#                        rospy.loginfo('EXCEPTION SO STOP!')
#                        self.tracker_command = 'STOP'

                if self.tracker_command in self.tracker_commands:
                    if self.tracker_command == 'STOP':
                        self.stop_joints()
                    else:
                        self.teleop_joints()        
                    self.cmd_joints.header.stamp = rospy.Time.now()        
                    #self.head_tilt_pub.publish(self.cmd_joints.position[self.cmd_joints.name.index('head_tilt_joint')])
                    #self.head_pan_pub.publish(self.cmd_joints.position[self.cmd_joints.name.index('head_pan_joint')])
                    self.left_shoulder_pan_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(left + '_shoulder_pan_joint')])
                    self.left_shoulder_lift_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(left + '_shoulder_lift_joint')])
                    self.left_arm_roll_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(left + '_arm_roll_joint')])
                    self.left_elbow_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(left + '_elbow_joint')])
                    self.left_forearm_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(left + '_forearm_joint')])
                    self.left_wrist_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(left + '_wrist_joint')])
                    #self.right_shoulder_pan_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(right + '_shoulder_pan_joint')])
                    #self.right_shoulder_lift_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(right + '_shoulder_lift_joint')])
                    self.right_arm_roll_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(right + '_arm_roll_joint')])
                    #self.right_elbow_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(right + '_elbow_joint')])
                    self.right_forearm_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(right + '_forearm_joint')])
                    #self.right_wrist_pub.publish(reflect * self.cmd_joints.position[self.cmd_joints.name.index(right + '_wrist_joint')])
                    #self.torso_pub.publish(self.cmd_joints.position[self.cmd_joints.name.index('torso_joint')])                
            
            self.last_cmd_joints = self.cmd_joints
            self.stop = True
            
            rate.sleep()
            
    def stop_joints(self):
        self.cmd_joints = self.last_cmd_joints
        
    def relax_joints(self):
        pass
        
    def reset_joints(self):
        pass
        
    def enable_joints(self):
        pass
            
    def teleop_joints(self):
        left = "left"
        right = "right"
        
        if self.mirror:
            reflect = -1
        else:
            reflect = 1
        
        self.cmd_joints.header.stamp = rospy.Time.now()
         
        # Fixed Joints: Set position to 0 radians.
        self.cmd_joints.position[self.cmd_joints.name.index(left + '_arm_roll_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_arm_roll_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index(right + '_arm_roll_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_arm_roll_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index(left + '_wrist_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_wrist_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index(right + '_wrist_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_wrist_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index(left + '_forearm_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_forearm_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index(right + '_forearm_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_forearm_joint')] = 2  
             
                    
        # Torso Rotation          
        try:
            torso_quaternion = self.skeleton['orientation']['torso']
            torso_rpy = torso_quaternion.GetRPY()
            #self.cmd_joints.position[self.cmd_joints.name.index('torso_joint')] = torso_rpy[1] / 2.0
            self.cmd_joints.position[self.cmd_joints.name.index('torso_joint')] = 0
            self.cmd_joints.velocity[self.cmd_joints.name.index('torso_joint')] = self.default_joint_speed
        except:
            pass
        
        # Head Pan           
        try:
            head_quaternion = self.skeleton['orientation']['head']
            head_rpy = head_quaternion.GetRPY()
            self.cmd_joints.position[self.cmd_joints.name.index('head_pan_joint')] = head_rpy[1]
            self.cmd_joints.velocity[self.cmd_joints.name.index('head_pan_joint')] = self.default_joint_speed
        except:
            pass
        
        # Head Tilt
        try:
            self.cmd_joints.position[self.cmd_joints.name.index('head_tilt_joint')] = head_rpy[0]
            self.cmd_joints.velocity[self.cmd_joints.name.index('head_tilt_joint')] = self.default_joint_speed
        except:
            pass
        
        # Left Arm
        try:
            left_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position'][left + '_shoulder']
            left_shoulder_elbow = self.skeleton['position'][left + '_elbow'] - self.skeleton['position'][left + '_shoulder']
            left_elbow_hand = self.skeleton['position'][left + '_hand'] - self.skeleton['position'][left + '_elbow']
            left_shoulder_hand = self.skeleton['position'][left + '_hand'] - self.skeleton['position'][left + '_shoulder']
            
            left_shoulder_neck.Normalize()
            left_shoulder_elbow.Normalize()
            lh = left_elbow_hand.Normalize()
            left_shoulder_hand.Normalize()                           

            left_shoulder_lift_angle = -asin(left_shoulder_elbow.y()) + pi / 3.0
            left_shoulder_pan_angle = -asin(left_elbow_hand.x()) + pi / 6.0
            
            left_elbow_angle = -acos(KDL.dot(left_shoulder_elbow, left_elbow_hand))

            left_wrist_angle = -left_elbow_angle / 2.0
            left_elbow_angle += pi / 6.0
            
            self.cmd_joints.position[self.cmd_joints.name.index(left + '_shoulder_lift_joint')] =  -self.HALF_PI + left_shoulder_lift_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_shoulder_lift_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index(left + '_shoulder_pan_joint')] = -self.HALF_PI / 3.0 + left_shoulder_pan_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_shoulder_pan_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index(left + '_elbow_joint')] = left_elbow_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_elbow_joint')] = self.default_joint_speed                                   
        
            self.cmd_joints.position[self.cmd_joints.name.index(left + '_wrist_joint')] = left_wrist_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_wrist_joint')] = self.default_joint_speed
            
#                left_elbow_rpy = [0]*3
#                left_elbow_rpy = euler_from_quaternion(self.skeleton['orientation'][left + '_elbow'])
#                left_arm_roll_angle = -left_elbow_rpy[2]

            left_arm_roll_angle = acos(KDL.dot(left_shoulder_elbow, left_shoulder_neck))
            #left_arm_roll_angle = -asin(left_shoulder_hand.x())
            self.cmd_joints.position[self.cmd_joints.name.index(left + '_arm_roll_joint')] = 0
            self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_arm_roll_joint')] = 2
        
#                left_wrist_angle = -acos(min(1, abs((lh - 0.265) / (0.30 - 0.265)))) + QUARTER_PI
#                self.cmd_joints.position[self.cmd_joints.name.index(left + '_wrist_joint')] = 0
#                self.cmd_joints.velocity[self.cmd_joints.name.index(left + '_wrist_joint')] = self.default_joint_speed
        
        except KeyError:
            pass      
            
        # Right Arm
        try:
            right_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position'][right + '_shoulder']
            right_shoulder_elbow = self.skeleton['position'][right + '_elbow'] - self.skeleton['position'][right + '_shoulder']
            right_elbow_hand = self.skeleton['position'][right + '_hand'] - self.skeleton['position'][right + '_elbow']
            right_shoulder_hand = self.skeleton['position'][right + '_hand'] - self.skeleton['position'][right + '_shoulder']
            
            right_shoulder_neck.Normalize()
            right_shoulder_elbow.Normalize()
            right_elbow_hand.Normalize()
            right_shoulder_hand.Normalize()                

            right_shoulder_lift_angle = asin(right_shoulder_elbow.y()) - pi / 3.0
            right_shoulder_pan_angle = -asin(right_elbow_hand.x()) + pi / 6.0
            
            right_elbow_angle = acos(KDL.dot(right_shoulder_elbow, right_elbow_hand))

            right_wrist_angle = -right_elbow_angle/ 2.0
            right_elbow_angle -= pi / 6.0
                                    
            self.cmd_joints.position[self.cmd_joints.name.index(right + '_shoulder_lift_joint')] = self.HALF_PI + right_shoulder_lift_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_shoulder_lift_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index(right + '_shoulder_pan_joint')] =  -self.HALF_PI / 3.0 + right_shoulder_pan_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_shoulder_pan_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index(right + '_elbow_joint')] = right_elbow_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_elbow_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index(right + '_wrist_joint')] = right_wrist_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_wrist_joint')] = self.default_joint_speed
 
            right_arm_roll_angle = -asin(right_shoulder_hand.x())
            self.cmd_joints.position[self.cmd_joints.name.index(right + '_arm_roll_joint')] = 0
            self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_arm_roll_joint')] = self.default_joint_speed
            
#                right_wrist_angle = acos(min(1, abs((rh - 0.265) / (0.30 - 0.265)))) - QUARTER_PI
#                self.cmd_joints.position[self.cmd_joints.name.index(right + '_wrist_joint')] = 0
#                self.cmd_joints.velocity[self.cmd_joints.name.index(right + '_wrist_joint')] = self.default_joint_speed
            
        except KeyError:
            pass          
            
    def skeleton_handler(self, msg):
        self.skeleton_lost = False
        self.cmd_joints.header.frame_id = msg.header.frame_id
        for joint in msg.name:
            self.skeleton['confidence'][joint] = msg.confidence[msg.name.index(joint)]
            self.skeleton['position'][joint] =  KDL.Vector(msg.position[msg.name.index(joint)].x, msg.position[msg.name.index(joint)].y, msg.position[msg.name.index(joint)].z)
            self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[msg.name.index(joint)].x, msg.orientation[msg.name.index(joint)].y, msg.orientation[msg.name.index(joint)].z, msg.orientation[msg.name.index(joint)].w)
            
    def joint_state_handler(self, msg):
        for joint in msg.name:  
            self.joint_state = msg
            
    def set_command_callback(self, req):
        self.tracker_command = req.command
        return SetCommandResponse()

    def shutdown(self):
        rospy.loginfo('Shutting down Tracker Joint Controller Node.')
        
if __name__ == '__main__':
    try:
        TrackerJointController()
    except rospy.ROSInterruptException:
        pass