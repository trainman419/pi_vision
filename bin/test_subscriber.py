#!/usr/bin/env python
"""
    A ROS listener node subscribing to the Serializer SensorState topic.
    
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
    
    http://www.gnu.org/licenses/gpl.
"""

import roslib; roslib.load_manifest('serializer')
import rospy
from serializer.msg import SensorState

def callback(data):
    rospy.loginfo(rospy.get_name() + " I heard %s", data.value)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("sensors", SensorState, callback)
    rospy.spin()

if __name__ == '__main__':
    
    listener()

