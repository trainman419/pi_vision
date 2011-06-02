#!/usr/bin/env python

"""
    fake_odometry.py - Version 1.0 2011-03-19
    
    Publish fake odometry data on the /odom topic
    
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

import roslib; roslib.load_manifest('pi_slam_tutorial')
import rospy

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from math import sin, cos


class OdomPublisher():
    def __init__(self):
        rospy.init_node('fake_odometry', anonymous=True)
        rate = rospy.get_param('~odom_rate', 20)
        r = rospy.Rate(rate)
        
        self.odom_frame = "/odom"        
        self.base_frame = "/base_link"
        
        # Set up the odometry publisher and broadcaster
        self.odom_pub = rospy.Publisher(self.odom_frame, Odometry)
        self.odom_broadcaster = TransformBroadcaster()
        
        x = 0.0
        y = 0.0
        th = 0.0
        
        vx = 0.2
        vy = 0.0
        vth = 0.4

        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
               
        rospy.loginfo("Starting fake odometry node at " + str(rate) + "Hz")
       
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            
            dx = (vx * cos(th) - vy * sin(th)) * dt
            dy = (vx * sin(th) + vy * cos(th)) * dt
            dth = vth * dt

            x += dx
            y += dy
            th += dth

            odom_quaternion = Quaternion()
            odom_quaternion.x = 0.0 
            odom_quaternion.y = 0.0
            odom_quaternion.z = sin(th / 2.0)
            odom_quaternion.w = cos(th / 2.0)
            
            self.odom_broadcaster.sendTransform((x, y, 0), 
                (odom_quaternion.x, odom_quaternion.y, odom_quaternion.z, odom_quaternion.w),
                rospy.Time.now(),
                self.base_frame,
                self.odom_frame)
        
            odom_message = Odometry()

            odom_message.header.stamp = current_time
            odom_message.header.frame_id = self.odom_frame
            odom_message.child_frame_id = self.base_frame

            odom_message.pose.pose.position.x = x
            odom_message.pose.pose.position.y = y
            odom_message.pose.pose.position.z = 0.0
            odom_message.pose.pose.orientation = odom_quaternion

            odom_message.twist.twist.linear.x = vx
            odom_message.twist.twist.linear.y = vy
            odom_message.twist.twist.angular.z = vth

            self.odom_pub.publish(odom_message)

            last_time = current_time
            r.sleep() 
        
if __name__ == '__main__':
    try:
        s = OdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down fake odometry node...")
        pass

