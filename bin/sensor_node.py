#!/usr/bin/env python

"""
    ROS Node for the Robotics Connection Serializer(TM) microcontroller
    
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

import roslib; roslib.load_manifest('serializer')
import rospy
import serializer_driver as SerializerAPI
from serializer.msg import SensorState
from serializer.srv import *
from base_controller import *
from geometry_msgs.msg import Twist
import threading, time

class Thread2(threading.Thread):
    def __init__(self, Serializer):
        threading.Thread.__init__(self)
        self.finished = threading.Event()
        self.interval = 0.05  # Access the voltage 2 times per second.
        self.daemon = False
        self.count = 0
        self.mySerializer = Serializer

    def run(self):
        max_delay = 0
        while not self.finished.isSet():
            start = datetime.now()
            output = "Thread 2:", self.mySerializer.mogo_m_per_s([1, 2], [0.07, -0.07])
            #output = "Thread 2:", self.mySerializer.rotate(30, 0.08)
            time.sleep(2)
            output = "Thread 2:", self.mySerializer.mogo_m_per_s([1, 2], [-0.07, 0.07])
            #output = "Thread 2:", self.mySerializer.rotate(-30, 0.08)
            time.sleep(2)
#            delay = (datetime.now() - start).microseconds / 1000
#            print output
#            if delay > max_delay:
#                max_delay = delay
#                print "MAX:", max_delay
            time.sleep(self.interval)
            
    def stop(self):
        print "Stopping Node 2 Thread ...",
        self.finished.set()
        self.join()
        print "Done."

class SerializerROS():
    def __init__(self):
        rospy.init_node('serializer')
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", "57600"))
        self.rate = int(rospy.get_param("~rate", "10"))
        self.publish_sensors = rospy.get_param("~publish_sensors", True)
        self.timeout = rospy.get_param("~timeout", "0.05")
                
        rospy.loginfo("Started Serializer on port " + self.port + " at " + str(self.baud))
        
        if self.publish_sensors:
            self.analog_sensors = dict({})
            self.digital_sensors = dict({})
            self.sensors = rospy.get_param("~sensors", dict({}))
            try:
                self.analog_sensors = self.sensors['analog']
            except:
                pass
            try:
                self.digital_sensors = self.sensors['digital']
            except:
                pass
            
            self.sensors = dict({})
            self.msg = SensorState()
        
            print "Publishing Sensors:"
            try:
                for sensor, params in self.analog_sensors.iteritems():
                    print sensor, params
            except:
                pass  
            try:      
                for sensor, params in self.digital_sensors.iteritems():
                    print sensor, params
            except:
                pass
            
            # The SensorState publisher
            self.sensorStatePub = rospy.Publisher('sensors', SensorState)
        
        # Subscribe to the velocity command topic so we can control the drive motors.
        #rospy.Subscriber('cmd_vel', Twist, self.cmdVelCallback)
        
        # The Serializer services.
        rospy.Service('SetServo', SetServo ,self.SetServoHandler)
        rospy.Service('Rotate', Rotate, self.RotateHandler)
        rospy.Service('TravelDistance', TravelDistance, self.TravelDistanceHandler)
        rospy.Service('GetAnalog', GetAnalog, self.GetAnalogHandler)
        rospy.Service('Ping', Ping, self.PingHandler)
        rospy.Service('Voltage', Voltage, self.VoltageHandler)
        rospy.Service('GP2D12', GP2D12, self.GP2D12Handler)
        rospy.Service('PhidgetsTemperature', PhidgetsTemperature, self.PhidgetsTemperatureHandler)
        rospy.Service('PhidgetsVoltage', PhidgetsVoltage, self.PhidgetsVoltageHandler)
        rospy.Service('PhidgetsCurrent', PhidgetsCurrent, self.PhidgetsCurrentHandler)
        
        self.rate = rospy.Rate(self.rate)
        
        # Initialize the Serializer driver
        self.mySerializer = SerializerAPI.Serializer(self.port, self.baud, self.timeout)
        self.mySerializer.connect()
        self.mySerializer.units = rospy.get_param("~units", 0)
        self.mySerializer.gear_reduction = rospy.get_param("~gear_reduction", 1.667)
        self.mySerializer.wheel_diameter = rospy.get_param("~wheel_diameter", 0.127)
        self.mySerializer.wheel_track = rospy.get_param("~wheel_track", 0.325)
        self.mySerializer.encoder_resolution = rospy.get_param("~encoder_resolution", 624)
              
        # Create the base controller.
        self.base = base_controller(self.mySerializer, "Serializer PID")
        self.base.start()
        
        
        while not rospy.is_shutdown():
            if self.publish_sensors:
                for sensor, params in self.analog_sensors.iteritems():
                    if params['type'] == "GP2D12":
                        self.sensors[sensor] = self.mySerializer.get_GP2D12(params['pin'])
                    elif params['type'] == "Voltage":
                        self.sensors[sensor] = self.mySerializer.voltage()
                    elif params['type'] == "PhidgetsCurrent":
                        self.sensors[sensor] = self.mySerializer.get_PhidgetsCurrent(params['pin'])
                    elif params['type'] == "PhidgetsVoltage":
                        self.sensors[sensor] = self.mySerializer.PhidgetsVoltage(params['pin'])
                    elif params['type'] == "PhidgetsTemperature":
                        self.sensors[sensor] = self.mySerializer.PhidgetsTemperature(params['pin'])
                    else:
                        self.sensors[sensor] = self.mySerializer.get_analog(params['pin'])
                    self.rate.sleep()
                        
                for sensor, params in self.digital_sensors.iteritems():
                    if params['type'] == "Ping":
                        self.sensors[sensor] = self.mySerializer.get_Ping(params['pin'])
                    else:
                        self.sensors[sensor] = self.mySerializer.get_io(params['pin'])
                    self.rate.sleep()

#                self.sensors['head_sonar'] = self.mySerializer.get_Ping(4)
#                time.sleep(0.05)
#                self.sensors['head_ir'] = self.mySerializer.get_analog(4)
                
                self.msg.name = list()
                self.msg.value = list()
                           
                for sensor, value in self.sensors.iteritems():
                    self.msg.name.append(sensor)
                    try:
                        self.msg.value.append(round(value, 1))
                    except:
                        self.msg.value.append(value)
                 
#                all_analog_sensors = self.mySerializer.sensor([1,2,3,4,5])       
#                left_encoder, right_encoder = self.mySerializer.get_encoder_count([1, 2])
#                self.msg.name.append("left_encoder")
#                self.msg.name.append("right_encoder")
#                self.msg.value.append(left_encoder)
#                self.msg.value.append(right_encoder)
               
                #self.msg.header.frame_id = "sensors"
                #self.msg.header.stamp = rospy.Time.now()
                #self.msg.header.seq += 1
                
                rospy.loginfo(self.msg)
                try:
                    self.sensorStatePub.publish(self.msg)
                except:
                    pass
                self.rate.sleep()
            else:
                rospy.spin()
            
    def SetServoHandler(self, req):
        self.mySerializer.servo(req.id, req.value)
        return SetServoResponse()
    
    def RotateHandler(self, req):
        self.mySerializer.rotate(req.angle, req.velocity)
        return RotateResponse()
    
    def TravelDistanceHandler(self, req):
        self.mySerializer.travel_distance(req.distance, req.velocity)
        return TravelDistanceResponse()
    
    def GetAnalogHandler(self, req):
        if req.cached is None:
            req.cached = False
        return GetAnalogResponse(self.mySerializer.get_analog(req.pin, req.cached))
    
    def PingHandler(self, req):
        return PingResponse(self.mySerializer.get_Ping(req.pin, req.cached))

    def GP2D12Handler(self, req):
        return GP2D12Response(self.mySerializer.get_GP2D12(req.pin, req.cached))
    
    def VoltageHandler(self):
        return VoltageResponse(self.mySerializer.voltage(req.cached))

    def PhidgetsTemperatureHandler(self, req):
        return PhidgetsTemperatureResponse(self.mySerializer.get_PhidgetsTemperature(req.pin, req.cached))
    
    def PhidgetsVoltageHandler(self, req):
        return PhidgetsVoltageResponse(self.mySerializer.get_PhidgetsVoltage(req.pin, req.cached))
    
    def PhidgetsCurrentHandler(self, req):
        return PhidgetsCurrentResponse(self.mySerializer.get_PhidgetsCurrent(req.pin, req.cached))

    def cmdVelCallback(self, req):
        """ Handle velocity-based movement requests. """
        x = req.linear.x        # m/s
        th = req.angular.z      # rad/s

        if x == 0:
            # Turn in place
            right = th * self.mySerializer.wheel_track  * self.mySerializer.gear_reduction / 2.0
            if th == 0:
                left = right
            else:
                left = -right
        elif th == 0:   
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            left = x - th * self.mySerializer.wheel_track  * self.mySerializer.gear_reduction / 2.0
            right = x + th * self.mySerializer.wheel_track  * self.mySerializer.gear_reduction / 2.0
            #d = x/th
            #l = x + th * (d - self.wheel_track/2.0)
            #r = x + th * (d + self.wheel_track/2.0)

        # Log motion.                  
        #rospy.loginfo("Twist move: " + str(left) + ", " + str(right))
        
        # Set motor speeds in meters per second.
        msg = "Left: " + str(left) + " Right: " + str(right)
        rospy.loginfo(msg)
        self.mySerializer.mogo_m_per_s([1, 2], [left, right])
        
           
if __name__ == '__main__':
    try:
        mySerializer = SerializerROS()
    except rospy.ROSInterruptException:
        mogoThread.stop()
        pass
        #self.base.stop()
