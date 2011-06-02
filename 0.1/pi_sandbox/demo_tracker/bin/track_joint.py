#!/usr/bin/env python

"""
    Track a given skeleton joint by panning and tilting the Kinect.
    
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
from sensor_msgs.msg import JointState, CameraInfo
from demo_tracker.msg import Skeleton
from demo_tracker.srv import *
from std_msgs.msg import Float64
import PyKDL as KDL
from ax12_controller_core.srv import SetSpeed
from math import acos, asin, atan, atan2, pi, radians

class TrackJoint():
    def __init__(self):
        rospy.init_node('tracker_joint_controller')
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Head Tracking Node...")
        
        self.image_width = 320
        self.image_height = 240
        
        self.rate = rospy.get_param('~head_tracking_rate', 5)
        r = rospy.Rate(self.rate)
        
        self.joint_to_track = rospy.get_param('~joint_to_track', 'head')

        self.mirror_skeleton = rospy.get_param('~mirror_skeleton', True)

        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.3)
        self.max_head_speed = rospy.get_param('~max_head_speed', 1.0)
        self.tracker_commands = rospy.get_param('~tracker_commands', ['STOP', 'TELEOP_JOINTS'])
        
        self.servo_speed = dict()
        ax12_namespace = '/ax12_controller'
        dynamixels = rospy.get_param(ax12_namespace+'/dynamixels', dict())
        for name in sorted(dynamixels):
            rospy.loginfo(name)
            try:
                rospy.wait_for_service(ax12_namespace+'/'+name+'_controller/set_speed')  
                self.servo_speed[name] = rospy.ServiceProxy(ax12_namespace+'/'+name+'_controller/set_speed', SetSpeed)
                self.servo_speed[name](self.default_joint_speed)
            except:
                pass
        if self.mirror_skeleton:
            self.mirror = -1.0
        else:
            self.mirror = 1.0
         
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

        # The joints are controlled by publishing joint positions on the ax12 controller topics.
        self.head_tilt_pub = rospy.Publisher('/ax12_controller/head_tilt_controller/command', Float64)
        self.head_pan_pub = rospy.Publisher('/ax12_controller/head_pan_controller/command', Float64)
#        self.left_shoulder_pan_pub = rospy.Publisher('/ax12_controller/left_shoulder_pan_controller/command', Float64)
#        self.left_shoulder_lift_pub = rospy.Publisher('/ax12_controller/left_shoulder_lift_controller/command', Float64)
#        self.left_arm_roll_pub = rospy.Publisher('/ax12_controller/left_arm_roll_controller/command', Float64)
#        self.left_elbow_pub = rospy.Publisher('/ax12_controller/left_elbow_controller/command', Float64)
#        self.left_forearm_pub = rospy.Publisher('/ax12_controller/left_forearm_controller/command', Float64)
#        self.left_wrist_pub = rospy.Publisher('/ax12_controller/left_wrist_controller/command', Float64)
#        self.right_shoulder_pan_pub = rospy.Publisher('/ax12_controller/right_shoulder_pan_controller/command', Float64)
#        self.right_shoulder_lift_pub = rospy.Publisher('/ax12_controller/right_shoulder_lift_controller/command', Float64)
#        self.right_arm_roll_pub = rospy.Publisher('/ax12_controller/right_arm_roll_controller/command', Float64)
#        self.right_elbow_pub = rospy.Publisher('/ax12_controller/right_elbow_controller/command', Float64)
#        self.right_forearm_pub = rospy.Publisher('/ax12_controller/right_forearm_controller/command', Float64)
#        self.right_wrist_pub = rospy.Publisher('/ax12_controller/right_wrist_controller/command', Float64)
#        self.torso_pub = rospy.Publisher('/ax12_controller/torso_controller', Float64)
        
        # The get_joints command parses a URDF description of the robot to get all the non-fixed joints.
        self.cmd_joints = JointState()
        self.cmd_joints = PTL.get_joints()
        
        # Store the last joint command so we can stop and hold a given posture.
        self.last_cmd_joints = JointState()
        self.last_cmd_joints = PTL.get_joints()
        
        # Initialize the robot in the stopped state.
        self.tracker_command = "STOP"
        
        """ The pan/tilt thresholds indicate how many pixels the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(rospy.get_param("~pan_threshold", 5))
        self.tilt_threshold = int(rospy.get_param("~tilt_threshold", 5))
        
        """ The k_pan and k_tilt parameter determine how responsive the servo movements are.
            If these are set too high, oscillation can result. """
        self.k_pan = rospy.get_param("~k_pan", 0.5)
        self.k_tilt = rospy.get_param("~k_tilt", 0.5)
        
        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param("~min_pan", radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))
        
        """ Use a JointState message to store the pan and tilt speeds """
        self.head_cmd = JointState()
        self.head_cmd.position = [0, 0]
        self.head_cmd.velocity = [1, 1]
        
        """ Center the head and pan servos at the start. """
        self.servo_speed['head_pan'](self.head_cmd.velocity[0])
        self.servo_speed['head_tilt'](self.head_cmd.velocity[1])
        self.head_pan_pub.publish(self.head_cmd.position[0])
        self.head_tilt_pub.publish(self.head_cmd.position[1])
        
        rospy.sleep(5)  
        rospy.loginfo("READY TO TRACK!")
        
        self.tracking_seq = 0
        self.last_tracking_seq = 0
        
        #rospy.Subscriber('/camera/camera_info', CameraInfo, self.getCameraInfo)
        
        while not rospy.is_shutdown():
            if self.last_tracking_seq == self.tracking_seq:
                rospy.loginfo("LOST SKELETON!")
                self.head_cmd.velocity = [0.01, 0.01]        
                #self.head_cmd.position = [0, 0]
            else:
                self.last_tracking_seq = self.tracking_seq
            self.servo_speed['head_pan'](self.head_cmd.velocity[0])
            self.servo_speed['head_tilt'](self.head_cmd.velocity[1])
            self.head_pan_pub.publish(self.head_cmd.position[0])
            self.head_tilt_pub.publish(self.head_cmd.position[1])
            rospy.loginfo(self.head_cmd)
            r.sleep()
            
#    def getCameraInfo(self, msg):
#        self.image_width = msg.width
#        self.image_height = msg.height

    def stop_joints(self):
        self.cmd_joints = self.last_cmd_joints
        
    def relax_joints(self):
        pass
        
    def reset_joints(self):
        pass
        
    def enable_joints(self):
        pass
            
    def teleop_joints(self):
        # Fixed Joints: Set position to 0 radians.
        self.cmd_joints.position[self.cmd_joints.name.index('left_arm_roll_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_roll_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('right_arm_roll_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_roll_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('left_wrist_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('left_wrist_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('right_wrist_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('right_wrist_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('left_forearm_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('left_forearm_joint')] = 2
        
        self.cmd_joints.position[self.cmd_joints.name.index('right_forearm_joint')] = 0
        self.cmd_joints.velocity[self.cmd_joints.name.index('right_forearm_joint')] = 2  
             
                    
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
            left_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position']['left_shoulder']
            left_shoulder_elbow = self.skeleton['position']['left_elbow'] - self.skeleton['position']['left_shoulder']
            left_elbow_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_elbow']
            left_shoulder_hand = self.skeleton['position']['left_hand'] - self.skeleton['position']['left_shoulder']
            
            left_shoulder_neck.Normalize()
            left_shoulder_elbow.Normalize()
            lh = left_elbow_hand.Normalize()
            left_shoulder_hand.Normalize()                           

            left_shoulder_lift_angle = -asin(left_shoulder_elbow.y()) + pi / 3.0           
            left_shoulder_pan_angle = -asin(left_elbow_hand.x()) + pi / 6.0
            left_elbow_angle = -acos(KDL.dot(left_shoulder_elbow, left_elbow_hand))

            left_wrist_angle = -left_elbow_angle / 2.0
            left_elbow_angle += pi / 6.0
            
            self.cmd_joints.position[self.cmd_joints.name.index('left_shoulder_lift_joint')] = -self.HALF_PI + left_shoulder_lift_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_shoulder_lift_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('left_shoulder_pan_joint')] = -self.HALF_PI / 3.0 + left_shoulder_pan_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_shoulder_pan_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('left_elbow_joint')] = left_elbow_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_elbow_joint')] = self.default_joint_speed                                   
        
            self.cmd_joints.position[self.cmd_joints.name.index('left_wrist_joint')] = left_wrist_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_wrist_joint')] = self.default_joint_speed
            
#                left_elbow_rpy = [0]*3
#                left_elbow_rpy = euler_from_quaternion(self.skeleton['orientation']['left_elbow'])
#                left_arm_roll_angle = -left_elbow_rpy[2]

            left_arm_roll_angle = acos(KDL.dot(left_shoulder_elbow, left_shoulder_neck))
            #left_arm_roll_angle = -asin(left_shoulder_hand.x())
            self.cmd_joints.position[self.cmd_joints.name.index('left_arm_roll_joint')] = 0
            self.cmd_joints.velocity[self.cmd_joints.name.index('left_arm_roll_joint')] = 2
        
#                left_wrist_angle = -acos(min(1, abs((lh - 0.265) / (0.30 - 0.265)))) + QUARTER_PI
#                self.cmd_joints.position[self.cmd_joints.name.index('left_wrist_joint')] = 0
#                self.cmd_joints.velocity[self.cmd_joints.name.index('left_wrist_joint')] = self.default_joint_speed
        
        except KeyError:
            pass      
            
        # Right Arm
        try:
            right_shoulder_neck = self.skeleton['position']['neck'] - self.skeleton['position']['right_shoulder']
            right_shoulder_elbow = self.skeleton['position']['right_elbow'] - self.skeleton['position']['right_shoulder']
            right_elbow_hand = self.skeleton['position']['right_hand'] - self.skeleton['position']['right_elbow']
            right_shoulder_hand = self.skeleton['position']['right_hand'] - self.skeleton['position']['right_shoulder']
            
            right_shoulder_neck.Normalize()
            right_shoulder_elbow.Normalize()
            right_elbow_hand.Normalize()
            right_shoulder_hand.Normalize()                

            right_shoulder_lift_angle = asin(right_shoulder_elbow.y()) - pi / 3.0
            right_shoulder_pan_angle = -asin(right_elbow_hand.x()) + pi / 6.0
            right_elbow_angle = acos(KDL.dot(right_shoulder_elbow, right_elbow_hand))

            right_wrist_angle = -right_elbow_angle/ 2.0
            right_elbow_angle -= pi / 6.0
                                    
            self.cmd_joints.position[self.cmd_joints.name.index('right_shoulder_lift_joint')] = self.HALF_PI + right_shoulder_lift_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_shoulder_lift_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('right_shoulder_pan_joint')] = -self.HALF_PI / 3.0 + right_shoulder_pan_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_shoulder_pan_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('right_elbow_joint')] = right_elbow_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_elbow_joint')] = self.default_joint_speed
            
            self.cmd_joints.position[self.cmd_joints.name.index('right_wrist_joint')] = right_wrist_angle
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_wrist_joint')] = self.default_joint_speed
 
            right_arm_roll_angle = -asin(right_shoulder_hand.x())
            self.cmd_joints.position[self.cmd_joints.name.index('right_arm_roll_joint')] = 0
            self.cmd_joints.velocity[self.cmd_joints.name.index('right_arm_roll_joint')] = self.default_joint_speed
            
#                right_wrist_angle = acos(min(1, abs((rh - 0.265) / (0.30 - 0.265)))) - QUARTER_PI
#                self.cmd_joints.position[self.cmd_joints.name.index('right_wrist_joint')] = 0
#                self.cmd_joints.velocity[self.cmd_joints.name.index('right_wrist_joint')] = self.default_joint_speed
            
        except KeyError:
            pass          
            
    def skeleton_handler(self, msg):
        self.skeleton = dict()
        self.skeleton['confidence'] = dict()
        self.skeleton['position'] = dict()
        self.skeleton['orientation'] = dict()
        self.cmd_joints.header.frame_id = msg.header.frame_id
        confident = False
        for joint in msg.name:
            self.skeleton['confidence'][joint] = msg.confidence[msg.name.index(joint)]
            if joint == 'neck' and self.skeleton['confidence'][joint] > 0.5:
                confident = True
            self.skeleton['position'][joint] =  KDL.Vector(msg.position[msg.name.index(joint)].x, msg.position[msg.name.index(joint)].y, msg.position[msg.name.index(joint)].z)
            self.skeleton['orientation'][joint] = KDL.Rotation.Quaternion(msg.orientation[msg.name.index(joint)].x, msg.orientation[msg.name.index(joint)].y, msg.orientation[msg.name.index(joint)].z, msg.orientation[msg.name.index(joint)].w)          
        
        """ Check to see if we have lost the skeleton. """
        if not confident:
            rospy.loginfo("LOST SKELETON!")
            #self.head_cmd.position = [0, 0]
            self.head_cmd.velocity = [0.01, 0.01]
            return
            
        """Use this counter to determine when we still have the skeleton """
        self.tracking_seq += 1

        """ Compute the center of the ROI """
        COG_x = -(self.skeleton['position'][self.joint_to_track].x() * 1000 - self.image_width / 2)
        COG_y = -(self.skeleton['position'][self.joint_to_track].y() * 1000 - self.image_height / 2)
        
        #rospy.loginfo("COG_X, COG_Y: " + str(COG_x) + " " + str(COG_y))
        #rospy.loginfo("X, Y: " + str(self.skeleton['position'][self.joint_to_track].x()) + " " + str(self.skeleton['position'][self.joint_to_track].y()))
          
        """ Pan the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_x) > self.pan_threshold:
            """ Set the pan speed proportion to the displacement of the horizontal displacement
                of the target. """
            self.head_cmd.velocity[0] = min(self.max_head_speed, self.k_pan * abs(COG_x) / float(self.image_width))
               
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_x > 0:
                self.head_cmd.position[0] = self.min_pan
            else:
                self.head_cmd.position[0] = self.max_pan
        else:
            self.head_cmd.velocity[0] = 0.0
        
        """ Tilt the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_y) > self.tilt_threshold:
            """ Set the tilt speed proportion to the displacement of the vertical displacement
                of the target. """
            self.head_cmd.velocity[1] = min(self.max_head_speed, self.k_tilt * abs(COG_y) / float(self.image_height))
            
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_y < 0:
                self.head_cmd.position[1] = self.min_tilt
            else:
                self.head_cmd.position[1] = self.max_tilt
        else:
            self.head_cmd.velocity[1] = 0.0
            
    def joint_state_handler(self, msg):
        for joint in msg.name:  
            self.joint_state = msg
            
    def set_command_callback(self, req):
        self.tracker_command = req.command
        return SetCommandResponse()

    def shutdown(self):
        rospy.loginfo('Shutting down Track Joint Controller Node.')
        
if __name__ == '__main__':
    try:
        TrackJoint()
    except rospy.ROSInterruptException:
        pass