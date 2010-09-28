#!/usr/bin/env python

"""
    Base Controller Class for the Robotics Connection Serializer(TM) microcontroller
    Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.
    
    Created for The Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses
"""
import roslib; roslib.load_manifest('serializer')
import rospy

from threading import Thread, Event

from math import sin,cos,pi
from datetime import datetime

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import sys

import random

class base_controller(Thread):
    """ Controller to handle movement & odometry feedback for a differential 
            drive mobile base. """
    def __init__(self, Serializer, name):
        Thread.__init__ (self)
        self.finished = Event()
        # Handle for the Serializer
        self.mySerializer = Serializer

        # Parameters
        self.rate = float(rospy.get_param("~base_controller_rate", 20.0))
        self.ticks_meter = float(self.mySerializer.ticks_per_meter)
        self.wheel_track = float(self.mySerializer.wheel_track)
        self.gear_reduction = float(self.mySerializer.gear_reduction)
        
        # internal data        
        self.enc_left = 0            # encoder readings
        self.enc_right = 0
        self.x = 0.                  # position in xy plane
        self.y = 0.
        self.th = 0.                 # rotation in radians
        self.last_time = rospy.Time.now() # time for determining dx/dy

        # subscriptions
        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCallback)
        #rospy.Subscriber("cmd_pose", Pose, self.cmdPoseCallback)
        self.odomPub = rospy.Publisher('odom', Odometry)
        self.odomBroadcaster = TransformBroadcaster()
        
        rospy.loginfo("Started Base Controller '"+ name +"' for a base of " + str(self.wheel_track) + "m wide with " + str(self.ticks_meter) + " ticks per meter")

    def run(self):
        rosRate = rospy.Rate(self.rate)
        rospy.loginfo("Publishing Odometry data at: " + str(self.rate) + " Hz")
        
        old_left = old_right = 0
        bad_encoder_count = 0
        
        while not rospy.is_shutdown() and not self.finished.isSet():
            rosRate.sleep()
            current_time = rospy.Time.now()

            # read encoders
            try:
                left, right = self.mySerializer.get_encoder_count([1, 2])
            except:
                rospy.loginfo("Could not update encoders: " + str(bad_encoder_count))
                bad_encoder_count += 1
                continue

            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time
            
            # calculate odometry
            dleft = (left - self.enc_left) / self.ticks_meter
            dright = (right - self.enc_right) / self.ticks_meter
            
            self.enc_left = left
            self.enc_right = right
            
            dxy_ave = (dleft + dright) / 2.0
            dth = (dright - dleft) / self.wheel_track
            vxy = dxy_ave / dt
            vth = dth / dt

            if (dxy_ave != 0):
                dx = cos(dth) * dxy_ave
                dy = -sin(dth) * dxy_ave
                self.x += (cos(self.th) * dx - sin(self.th) * dy)
                self.y += (sin(self.th) * dx + cos(self.th) * dy)

            if (dth != 0):
                self.th += dth 

            quaternion = Quaternion()
            quaternion.x = 0.0 
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2.0)
            quaternion.w = cos(self.th / 2.0)

            # Create the odometry transform frame broadcaster.
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0), 
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                "base_link",
                "odom"
                )
            
#            self.odomBroadcaster.sendTransform(
#                (0, 0, 0), 
#                (0, 0, 0, 1),
#                rospy.Time.now(),
#                "odom",
#                "world"
#                )

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = current_time
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion

            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = vxy
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = vth
            
            if abs(vth) > 0.71 or abs(vxy) > 0.21:
                rospy.loginfo("dLeft: " + str(dleft) + " dRight: " + str(dright) + " dt: " + str(dt) + "ct: " + str(current_time) + " lt: " + str(self.last_time))
            
            #rospy.loginfo(odom)
            self.odomPub.publish(odom)

            

    def cmdVelCallback(self, req):
        """ Handle velocity-based movement requests. """
        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s
        if x > 0.3:
            rospy.loginfo("X MAX EXCEEDED! " + str(x))
        if th > 1.0:
            rospy.loginfo("THETA MAX EXCEEDED! " + str(th))
        #rospy.loginfo("x/th: " + str(x) + "/" + str(th))
        if x == 0:
            # Turn in place
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:   
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0
            #d = x/th
            #l = x + th * (d - self.wheel_track/2.0)
            #r = x + th * (d + self.wheel_track/2.0)

        # Log motion.                  
        #rospy.loginfo("Twist move: " + str(left) + ", " + str(right))
        
        # Set motor speeds in meters per second.
        #rospy.loginfo("")
        if left < 0 and right < 0:
            rospy.loginfo("GOING BACKWARDS!!!!!!!!!!!!!!!!!!!!!!!!")
        #rospy.loginfo("Left/Right: " + str(left) + "/" + str(right))
        self.mySerializer.mogo_m_per_s([1, 2], [left, right])
        
    def stop(self):
        print "Shutting down base controller"
        self.finished.set()
        self.join()
        

    

    
