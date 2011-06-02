#!/usr/bin/env python

"""
    Convert a skeleton transform tree to a list of visualization markers for RViz.
        
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

import roslib; roslib.load_manifest('skeleton_markers')
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import tf

class SkeletonMarkers():
    def __init__(self):
        rospy.init_node('skeleton_markers')
        
        rospy.on_shutdown(self.shutdown)
        
        rospy.loginfo("Initializing Skeleton Markers Node...")
        
        self.rate = rospy.get_param('~rate', 20)
        self.scale = rospy.get_param('~scale', 0.07)
        self.lifetime = rospy.get_param('~lifetime', 0) # 0 is forever
        self.ns = rospy.get_param('~ns', 'skeleton_markers')
        self.id = rospy.get_param('~id', 0)
        self.tf_prefix = rospy.get_param('~tf_prefix', 'tracker')
        self.camera_frame_id = rospy.get_param('~camera_frame_id', '/openni_depth_optical_frame')
        #self.camera_frame_id = '/' + self.tf_prefix + '/' + self.camera_frame_id
        self.color = rospy.get_param('~color', {'r': 0.0, 'g': 1.0, 'b': 0.0, 'a': 1.0})
        self.skeleton_frames = list()

        rate = rospy.Rate(self.rate)
        
        # Initialize tf listener
        self.tf = tf.TransformListener()
        
        # Make sure we can see at least the torso frame
        #self.tf.waitForTransform(self.camera_frame_id, 'torso', rospy.Time(), rospy.Duration(5.0))
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('skeleton_markers', Marker)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.header.frame_id = self.camera_frame_id
        self.markers.ns = self.ns
        self.markers.id = self.id
        self.markers.type = Marker.POINTS
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(self.lifetime)
        self.markers.scale.x = self.scale
        self.markers.scale.y = self.scale
        self.markers.color.r = self.color['r']
        self.markers.color.g = self.color['g']
        self.markers.color.b = self.color['b']
        self.markers.color.a = self.color['a']
        
                
        while not rospy.is_shutdown():
            # Get the list of all skeleton frames
            self.skeleton_frames = [f for f in self.tf.getFrameStrings() if f.startswith("/" + self.tf_prefix)]
            self.markers.header.stamp = rospy.Time.now()
            self.markers.points = list()
            for frame in self.skeleton_frames:
                if frame == self.camera_frame_id:
                    continue
                # Find the position of the frame's origin relative to the camera frame.
                try:
                    position = Point()
                    (trans, rot)  = self.tf.lookupTransform(self.camera_frame_id, frame, rospy.Time(0))
                    position.x = trans[0]
                    position.y = trans[1]
                    position.z = trans[2]
                    self.markers.points.append(position)
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    #rospy.logerr("tf error when looking up " + frame + ' and ' + self.camera_frame_id)
                    continue
            #rospy.loginfo(self.markers)
            self.marker_pub.publish(self.markers)                       
            rate.sleep()

    def shutdown(self):
        rospy.loginfo('Shutting down Skeleton Marker Node.')
        
if __name__ == '__main__':
    try:
        SkeletonMarkers()
    except rospy.ROSInterruptException:
        pass