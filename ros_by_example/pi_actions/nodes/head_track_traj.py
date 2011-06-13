#!/usr/bin/env python

"""
    head_track_traj.py - Version 1.0 2010-12-28
    
    Use ROS trajectories to track a target given by (x,y) coordinates
    
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

import roslib; roslib.load_manifest('pi_actions')
import rospy
from sensor_msgs.msg import RegionOfInterest, CameraInfo
from trajectory_msgs.msg import *
from math import radians
import time

class head_track():
    def __init__(self):
        rospy.init_node("head_track_traj")
        
        rospy.on_shutdown(self.shutdown)
        
        self.image_width = 640
        self.image_height = 480
        
        rate = rospy.get_param('~rate', 10)
        r = rospy.Rate(rate)
        
        self.traj_pub = rospy.Publisher('/head_traj_controller/command', JointTrajectory)
                
        self.trajectory = JointTrajectory()
        self.trajectory.header.stamp = rospy.Time.now()
        self.trajectory.header.frame_id = 'head_link'
        self.trajectory.joint_names = ['head_pan_joint', 'head_tilt_joint']
        self.trajectory.points.append(JointTrajectoryPoint(positions = [0, 0], velocities = [0.5, 0.5], time_from_start = rospy.Duration(0.0)))  
        """ The pan/tilt thresholds indicate how many pixels the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(rospy.get_param("~pan_threshold", 5))
        self.tilt_threshold = int(rospy.get_param("~tilt_threshold", 5))
        
        """ The k_pan and k_tilt parameter determine how responsive the servo movements are.
            If these are set too high, oscillation can result. """
        self.k_pan = rospy.get_param("~k_pan", 2.0)
        self.k_tilt = rospy.get_param("~k_tilt", 1.0)
        
        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param("~min_pan", radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))
    
        """ Center the head and pan servos at the start. """
        rospy.loginfo("Centering servos...")
        rospy.sleep(1)
        self.traj_pub.publish(self.trajectory)
        rospy.sleep(2)
        
        self.tracking_seq = 0
        self.last_tracking_seq = -1
        
        roi_sub = rospy.Subscriber('roi', RegionOfInterest, self.setPanTiltSpeeds)
        #camera_info_sub = rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.getCameraInfo)
        
        while not rospy.is_shutdown():
            """ Publish the pan/tilt movement commands. """
            if self.last_tracking_seq == self.tracking_seq:
                self.trajectory.points = list()
                self.trajectory.points.append(JointTrajectoryPoint(positions = [0, 0], velocities = [0.0001, 0.0001], time_from_start = rospy.Duration(0.0)))
            else:
                self.last_tracking_seq = self.tracking_seq
            
            self.traj_pub.publish(self.trajectory)
            r.sleep()
    
    def setPanTiltSpeeds(self, msg):
        """ Check to see if we have lost the ROI. """
        if msg.width == 0 or msg.height == 0 or msg.width > self.image_width / 2 or \
                msg.height > self.image_height / 2:
            return
        
        """ When OpenCV loses the ROI, the message stops updating.  Use this counter to
            determine when it stops. """
        self.tracking_seq += 1

        """ Compute the center of the ROI """
        COG_x = msg.x_offset + msg.width / 2 - self.image_width / 2
        COG_y = msg.y_offset + msg.height / 2 - self.image_height / 2
          
        """ Pan the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_x) > self.pan_threshold:
            """ Set the pan speed proportion to the displacement of the horizontal displacement
                of the target. """
            pan_speed = self.k_pan * abs(COG_x) / float(self.image_width)
        else:
            pan_speed = 0.0001
               
        """ Set the target position to one of the min or max positions--we'll never
            get there since we are tracking using speed. """
        if COG_x > 0:
            pan_position = self.min_pan
        else:
            pan_position  = self.max_pan

        """ Tilt the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_y) > self.tilt_threshold:
            """ Set the tilt speed proportion to the displacement of the vertical displacement
                of the target. """
            tilt_speed = self.k_tilt * abs(COG_y) / float(self.image_height)
        else:
            tilt_speed = 0.0001
            
        """ Set the target position to one of the min or max positions--we'll never
            get there since we are tracking using speed. """
        if COG_y > 0:
            tilt_position = self.min_tilt
        else:
            tilt_position = self.max_tilt


        self.trajectory.points = list()
        self.trajectory.points.append(JointTrajectoryPoint(positions = [pan_position, tilt_position], velocities = [pan_speed, tilt_speed], time_from_start = rospy.Duration(0.0)))
            
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        rospy.loginfo("HELLO")
        
    def shutdown(self):
        self.trajectory.header.stamp = rospy.Time.now()
        self.trajectory.points = list()
        self.trajectory.points.append(JointTrajectoryPoint(positions = [0, 0], velocities = [0.5, 0.5], time_from_start = rospy.Duration(0.0)))    
        self.traj_pub.publish(self.trajectory)
        rospy.loginfo("Shutting down head tracking node...")         
                   
if __name__ == '__main__':
    try:
        head_track()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node is shut down.")




