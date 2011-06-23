#!/usr/bin/env python

"""
    sample_node.py - This sample program subscribes to the /serializer/SensorState
    topic and echos the results.  It also moves a servo on GPIO pin 8 (servo ID 1)
    if the value of the first sensor (index 0) is above or below a certain threshold.
    
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

import roslib; roslib.load_manifest('pi_robot')
import rospy
from serializer.srv import *
from serializer.msg import *

class sample_node():
    def __init__(self):
        # Name this node
        rospy.init_node("sample_node")
        
        # Execute this function when shutting down
        rospy.on_shutdown(self.shutdown)
        
        # Set the update rate to 1 second by default
        self.rate = rospy.get_param("~node_rate", 1)
        r = rospy.Rate(self.rate)
        
        # Initialize the sensor value array
        self.sensor_state = SensorState()
        
        # Subscribe to the senors topic
        rospy.Subscriber('/serializer/sensors', SensorState, self.getSensorState)
        
        # Subscribe to the SetServo service
        rospy.wait_for_service('/serializer/SetServo')      
        self.servo_proxy = rospy.ServiceProxy('/serializer/SetServo', SetServo)

        # Begin the main loop
        while not rospy.is_shutdown():
            # Echo back the current sensor values
            rospy.loginfo(self.sensor_state)
            
            # Move one of the servos depending on the value of one of the sensors
            try:
                if self.sensor_state.value[0] > 6.4:
                    self.servo_proxy(1, 50)
                else:
                    self.servo_proxy(1, -50)
            except:
                pass
                
            # Sleep for 1/rate seconds
            r.sleep()
            
    # Callback for the sensor state subscriber
    def getSensorState(self, data):
        self.sensor_state = data
    
    # Shutdown function just prints a message.
    def shutdown(self):
        rospy.loginfo("Shutting down Sample Node...")

if __name__ == '__main__':
    try:
        my_node = sample_node()
    except rospy.ROSInterruptException:
        pass
