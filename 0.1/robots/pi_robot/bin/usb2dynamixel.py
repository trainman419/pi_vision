#!/usr/bin/env python

import roslib; roslib.load_manifest('robotis')
import rospy
from robotis.ros_robotis import *

poller = ROS_Robotis_Poller( '/dev/ttyUSB0', [1,2,3,4,5,13,6,7,8,9,12,10,11], ['head_pan_joint', 'head_tilt_joint', 'right_shoulder_lift_joint', 'right_shoulder_pan_joint', 'right_arm_roll_joint', 'right_elbow_joint', 'right_wrist_joint', 'left_shoulder_lift_joint', 'left_shoulder_pan_joint', 'left_arm_roll_joint', 'left_elbow_joint', 'left_wrist_joint','torso_joint'] )
rospy.spin()
poller.stop()