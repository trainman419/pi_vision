#!/usr/bin/env python

"""
    Point Cloud for Pi Robot
    
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
import math
from sensor_msgs.msg import PointCloud

rospy.init_node("point_cloud_publisher")
cloud_pub = rospy.Publisher("point_cloud", PointCloud)
rate = rospy.Rate(1.0)

num_points = 100
count = 0

while not rospy.is_shutdown():
    cloud = PointCloud()
    cloud.header.stamp = rospy.Time.now()
    cloud.header.frame_id = "sensor_frame"
    
    # We'll also add an intensity channel to the cloud
    cloud.channels.name = list()
    cloud.channels.name.append("intensities")
    cloud.channels[0].values = list()
    cloud.points.x = list()
    cloud.points.y = list()
    cloud.points.z = list()

    # Generate some fake data for our point cloud
    for i in range(num_points):
      cloud.points.x.append(1 + count)
      cloud.points.y.append(2 + count)
      cloud.points.z.append(3 + count)
      cloud.channels.values.append(100 + count)

    cloud_pub.publish(cloud)
    count += 1
    rate.sleep()
