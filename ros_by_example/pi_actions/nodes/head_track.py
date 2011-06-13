#!/usr/bin/env python

"""
    head_track.py - Version 1.0 2010-12-28
    
    Move the head to track a target given by (x,y) coordinates
    
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
from std_msgs.msg import Float64
from math import radians

class head_track():
    def __init__(self):
        rospy.init_node("head_track")
        rospy.on_shutdown(self.shutdown)
        
        self.image_width = 320
        self.image_height = 240
        
        servo_namespace = rospy.get_namespace()
        self.head_pan_controller_topic = rospy.get_param('head_pan_controller_topic', servo_namespace + 'head_pan_joint/command')
        self.head_tilt_controller_topic = rospy.get_param('head_tilt_controller_topic', servo_namespace + 'head_tilt_joint/command')
        self.head_pan_speed_param = rospy.get_param('head_pan_speed_param', '/arbotix/dynamixels/head_pan_joint/max_speed')
        self.head_tilt_speed_param = rospy.get_param('head_tilt_speed_param', '/arbotix/dynamixels/head_tilt_joint/max_speed')
        
        # Make sure the servos are initialized to slow max speeds.
        rospy.set_param(self.head_pan_speed_param, 25)
        rospy.set_param(self.head_tilt_speed_param, 25)
        
        self.rate = rospy.get_param("~rate", 20)
        r = rospy.Rate(self.rate)
        
        """ The pan/tilt thresholds indicate how many pixels the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(rospy.get_param("~pan_threshold", 5))
        self.tilt_threshold = int(rospy.get_param("~tilt_threshold", 5))
        
        """ The k_pan and k_tilt parameter determine how responsive the servo movements are.
            If these are set too high, oscillation can result. """
        self.k_pan = rospy.get_param("~k_pan", 3.5)
        self.k_tilt = rospy.get_param("~k_tilt", 2.5)
        
        self.max_pan = rospy.get_param("~max_pan", radians(145))
        self.min_pan = rospy.get_param("~min_pan", radians(-145))
        self.max_tilt = rospy.get_param("~max_tilt", radians(90))
        self.min_tilt = rospy.get_param("~min_tilt", radians(-90))
        
        """ Initialize publishers for the pan and tilt servos """
        self.head_pan_pub = rospy.Publisher(self.head_pan_controller_topic, Float64)
        self.head_tilt_pub = rospy.Publisher(self.head_tilt_controller_topic, Float64)
    
        """ Center the head and pan servos at the start. """
        self.target_pan = 0.0
        self.target_tilt = 0.0
        
        rospy.loginfo("Centering servos...")
        self.head_pan_pub.publish(self.target_pan)
        self.head_tilt_pub.publish(self.target_tilt)

        rospy.sleep(2)
        
        self.tracking_seq = 0
        self.last_tracking_seq = -1
        
        rospy.Subscriber('roi', RegionOfInterest, self.setPanTiltSpeeds)
        #rospy.Subscriber('/camera/camera_info', CameraInfo, self.getCameraInfo)
        
        while not rospy.is_shutdown():
            """ Publish the pan/tilt movement commands. """
            if self.last_tracking_seq == self.tracking_seq:
                rospy.set_param(self.head_pan_speed_param, 0)
                rospy.set_param(self.head_tilt_speed_param, 0)
            else:
                self.last_tracking_seq = self.tracking_seq
            
            self.head_pan_pub.publish(self.target_pan)
            self.head_tilt_pub.publish(self.target_tilt)
            #rospy.loginfo("Track: " + str(self.target_pan) + " " + str(self.target_tilt))
            r.sleep()
    
    def setPanTiltSpeeds(self, msg):
        """ When OpenCV loses the ROI, the message stops updating.  Use this counter to
            determine when it stops. """
        self.tracking_seq += 1
        
        """ Check to see if we have lost the ROI. """
        if msg.width == 0 or msg.height == 0 or msg.width > self.image_width / 2 or \
                msg.height > self.image_height / 2:
            rospy.set_param(self.head_pan_speed_param, 0)
            rospy.set_param(self.head_tilt_speed_param, 0)
            return

        """ Compute the center of the ROI """
        COG_x = msg.x_offset + msg.width / 2 - self.image_width / 2
        COG_y = msg.y_offset + msg.height / 2 - self.image_height / 2
        rospy.loginfo("COG_x: " + str(COG_x) + " COG_y: " + str(COG_y))
          
        """ Pan the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_x) > self.pan_threshold:
            """ Set the pan speed proportion to the displacement of the horizontal displacement
                of the target. """
            pan_speed = self.k_pan * abs(COG_x) / float(self.image_width)
            pan_speed = 0.001
            rospy.set_param(self.head_pan_speed_param, pan_speed)
               
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_x > 0:
                self.target_pan = self.min_pan
            else:
                self.target_pan = self.max_pan
        else:
            pan_speed = 0
            rospy.set_param(self.head_pan_speed_param, pan_speed)
        
        """ Tilt the camera only if the displacement of the COG exceeds the threshold. """
        if abs(COG_y) > self.tilt_threshold:
            """ Set the tilt speed proportion to the displacement of the vertical displacement
                of the target. """
            tilt_speed = self.k_tilt * abs(COG_y) / float(self.image_height)
            tilt_speed = 0.001
            rospy.set_param(self.head_tilt_speed_param, tilt_speed)
            
            """ Set the target position to one of the min or max positions--we'll never
                get there since we are tracking using speed. """
            if COG_y > 0:
                self.target_tilt = self.min_tilt
            else:
                self.target_tilt = self.max_tilt
        else:
            tilt_speed = 0
            rospy.set_param(self.head_tilt_speed_param, tilt_speed)
            
        rospy.loginfo("Pan spd: " + str(pan_speed) + " Tilt spd: " + str(tilt_speed))
            
    def getCameraInfo(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height
        
    def shutdown(self):
        self.target_pan = 0.0
        self.target_tilt = 0.0
        self.head_pan_pub.publish(self.target_pan)
        self.head_tilt_pub.publish(self.target_tilt)
        rospy.loginfo("Shutting down head tracking node...")         
                   
if __name__ == '__main__':
    try:
        head_track()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node is shut down.")




