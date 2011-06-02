#!/usr/bin/env python

""" 
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

import roslib; roslib.load_manifest('pi_demo')
import rospy
import tf
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

class TrackRightHand():
    def __init__(self):
        
        # Initialize new node
        rospy.init_node('track_right_hand', anonymous=True)
        
        rate = rospy.get_param('~rate', 1)
        self.fixed_frame = rospy.get_param('~fixed_frame', 'kinect_depth_frame')
        self.target_frame = rospy.get_param('~target_frame', '/tracker/right_hand')

        r = rospy.Rate(1)
        
        # Initialize the target point
        self.target_point = PointStamped()
        self.last_target_point = PointStamped()
        
        self.target_point.header.frame_id = self.target_frame

        # Create the target point publisher
        self.targetPub = rospy.Publisher('/target_point', PointStamped)

        # Initialize tf listener
        self.tf = tf.TransformListener()
        
        while not rospy.is_shutdown():
            try:
                position = Point()
                (trans, rot)  = self.tf.lookupTransform(self.fixed_frame, self.target_frame, rospy.Time(0))
                position.x = trans[0]
                position.y = trans[1]
                position.z = trans[2]
                self.target_point.header.stamp = rospy.Time.now()
                self.target_point.point.x = position.x
                self.target_point.point.y = position.y
                self.target_point.point.z = position.z
                rospy.loginfo("Publishing target point:\n" + str(self.target_point))
                self.targetPub.publish(self.target_point)
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("tf error when looking up " + self.target_frame + ' and ' + self.fixed_frame)
                                    
            r.sleep()

    def shutdown(self):
        rospy.loginfo('Shutting down Right Hand Tracker Node.')
        
if __name__ == '__main__':
    try:
        TrackRightHand()
    except rospy.ROSInterruptException:
        pass