#!/usr/bin/env python

"""
    base_scan.py - A panning sonar sensor used to get a pretend "laser scan" for ROS SLAM
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
    
    NOTE: This code borrows heavily from Michael Ferguson's Poor Man's Lidar code.
"""

import roslib; roslib.load_manifest('pi_robot')
import rospy
from sensor_msgs.msg import LaserScan
from tf.broadcaster import TransformBroadcaster
from serializer.srv import *
from serializer.msg import *
from math import pi
import time
from threading import Thread, Event


class base_scan(Thread):
    def __init__(self, Serializer, name):
        Thread.__init__ (self)
        self.finished = Event()
        #rospy.init_node("base_scan")
        self.scanPub = rospy.Publisher('base_scan', LaserScan)
        self.scanBroadcaster = TransformBroadcaster()
        #rospy.Subscriber("sensors", SensorState, self.sensorCallback)
        
        self.rate = 1        
        rospy.loginfo("Started base sonar scan at " + str(self.rate) + " Hz")
        
        self.sonar = 0.128  
     
        # Scanning servo
        self.servo_id = 2
        self.sensor_id = 5
        self.right = 60
        self.left = -80
#        self.right = 70
#        self.left = -90
        self.sweep_angle = pi * (self.right - self.left) / 200.
        self.angle_max = self.sweep_angle / 2.0
        self.angle_min = -self.angle_max
        self.n_samples = 20
        self.angle_increment = self.sweep_angle / self.n_samples
        self.servo_increment = (self.right - self.left) / self.n_samples
        self.range_min = 0.02
        self.range_max = 3.0

        self.setServo(self.servo_id, (self.right - self.left) / 2 + self.left)
        self.servo_direction = 1
        self.count = 0
        
        sonar = None
        while sonar == None:
            sonar = self.getPing(self.sensor_id, False)
        last_sonar = sonar
        rospy.loginfo("Initial Sonar Reading: " + str(sonar))
        
    def run(self):
        rosRate = rospy.Rate(self.rate)
        rospy.loginfo("Executing base scan at: " + str(self.rate) + " Hz")
        
        while not rospy.is_shutdown():
            
            ranges = list()

            for i in range(self.n_samples):
                sonar = self.getPing(self.sensor_id, False)
                ranges.append(sonar / 100.0)
#                if self.servo_direction > 0:
#                    self.setServo(self.servo_id, self.left + i * self.servo_increment)
#                else:
#                    self.setServo(self.servo_id, self.right - i * self.servo_increment)
                time.sleep(0.05)
                        
            if self.servo_direction > 0:
                ranges.reverse()
                
            scan = LaserScan()
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id = "base_scan"
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.scan_time = self.rate
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            scan.ranges = ranges    
            self.scanPub.publish(scan)
            
            self.servo_direction *= -1
            
        #    scan = LaserScan()
        #    scan.header.stamp = rospy.Time.now()        
        #    scan.header.frame_id = "base_link"
        #    scan.angle_min = -1.57
        #    scan.angle_max = 1.57
        #    scan.angle_increment = 0.108275862
        #    scan.scan_time = scan_rate
        #    scan.range_min = 0.5
        #    scan.range_max = 6.0
        #    scan.ranges = ranges    
        #    scanPub.publish(scan)
        
            rosRate.sleep()
            
    def stop(self):
        print "Shutting down base scan"
        self.finished.set()
        self.join()

    def setServo(self, id, position):
        rospy.wait_for_service('SetServo')
        try:
            servoProxy = rospy.ServiceProxy('SetServo', SetServo)
            return servoProxy(id, position) 
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def getPing(self, id, cached):
        rospy.wait_for_service('Ping')
        try:
            pingProxy = rospy.ServiceProxy('Ping', Ping)
            response = pingProxy(id, cached)
            return response.value
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            
    def getVoltage(self, cached):
        rospy.wait_for_service('Voltage')
        try:
            voltageProxy = rospy.ServiceProxy('Voltage', Voltage)
            response = voltageProxy(cached)
            voltage = response.value
            return voltage
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
    def sensorCallback(self, data):
        self.sonar = data.value[0] / 100.
        #rospy.loginfo("Sensor Data: " + str(data.value))
    

if __name__ == '__main__':
    try:
        myBaseScan = base_scan()
        myBaseScan.start()
    except rospy.ROSInterruptException:
        pass
