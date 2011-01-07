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
import os

class SerializerROS():
    def __init__(self):
        rospy.init_node('serializer')
        rospy.on_shutdown(self.shutdown)
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baud = int(rospy.get_param("~baud", 19200))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.publish_sensors = rospy.get_param("~publish_sensors", False)
        self.rate = int(rospy.get_param("~sensor_rate", 10))
        
        self.pid_params = dict()
        self.pid_params['use_base_controller'] = rospy.get_param("~use_base_controller", False)
        self.pid_params['units'] = rospy.get_param("~units", 0)
        self.pid_params['wheel_diameter'] = rospy.get_param("~wheel_diameter", "") 
        self.pid_params['wheel_track'] = rospy.get_param("~wheel_track", "")
        self.pid_params['encoder_type'] = rospy.get_param("~encoder_type", 1) 
        self.pid_params['encoder_resolution'] = rospy.get_param("~encoder_resolution", "") 
        self.pid_params['gear_reduction'] = rospy.get_param("~gear_reduction", 1.0)
        self.pid_params['motors_reversed'] = rospy.get_param("~motors_reversed", False)
        
        self.pid_params['init_pid'] = rospy.get_param("~init_pid", False)  
        self.pid_params['VPID_P'] = rospy.get_param("~VPID_P", "")
        self.pid_params['VPID_I'] = rospy.get_param("~VPID_I", "")
        self.pid_params['VPID_D']  = rospy.get_param("~VPID_D", "")
        self.pid_params['VPID_L'] = rospy.get_param("~VPID_L", "")
        self.pid_params['DPID_P'] = rospy.get_param("~DPID_P", "")
        self.pid_params['DPID_I'] = rospy.get_param("~DPID_I", "")
        self.pid_params['DPID_D'] = rospy.get_param("~DPID_D", "")
        self.pid_params['DPID_A'] = rospy.get_param("~DPID_A", "")
        self.pid_params['DPID_B'] = rospy.get_param("~DPID_B", "")
        
        # Check PID parameters if we are using the base controller.
        if self.pid_params['use_base_controller']:
            pid_error = False
            for param in self.pid_params:
                if self.pid_params[param] == "":
                    rospy.logerr("*** PID Parameter " + param + " is missing. ***")
                    pid_error = True
            if pid_error:
                rospy.signal_shutdown("Missing PID parameters.")
                os._exit(1)
                
        rospy.loginfo("Connected to Serializer on port " + self.port + " at " + str(self.baud) + " baud")
        rospy.loginfo("Publishing Serializer data at " + str(self.rate) + " Hz")
        
        if self.publish_sensors:
            try:
                self.analog_sensors = rospy.get_param("~analog", dict({}))
            except:
                pass
            try:
                self.digital_sensors = rospy.get_param("~digital", dict({}))
            except:
                pass
        
            rospy.loginfo("Publishing Sensors:")
            try:
                for sensor, params in self.analog_sensors.iteritems():
                    rospy.loginfo(sensor + " " + str(params))
            except:
                pass  
            try:      
                for sensor, params in self.digital_sensors.iteritems():
                    rospy.loginfo(sensor + " " + str(params))
            except:
                pass
            
            # The SensorState publisher
            self.sensorStatePub = rospy.Publisher('~sensors', SensorState)
        
        # The Serializer services.
        rospy.Service('~SetServo', SetServo ,self.SetServoHandler)
        rospy.Service('~GetAnalog', GetAnalog, self.GetAnalogHandler)
        rospy.Service('~Voltage', Voltage, self.VoltageHandler)
        rospy.Service('~Ping', Ping, self.PingHandler)
        rospy.Service('~GP2D12', GP2D12, self.GP2D12Handler)
        rospy.Service('~PhidgetsTemperature', PhidgetsTemperature, self.PhidgetsTemperatureHandler)
        rospy.Service('~PhidgetsVoltage', PhidgetsVoltage, self.PhidgetsVoltageHandler)
        rospy.Service('~PhidgetsCurrent', PhidgetsCurrent, self.PhidgetsCurrentHandler)
        rospy.Service('~Rotate', Rotate, self.RotateHandler)
        rospy.Service('~TravelDistance', TravelDistance, self.TravelDistanceHandler)
   
        rosRate = rospy.Rate(self.rate)
        
        # Initialize the Serializer driver
        self.mySerializer = SerializerAPI.Serializer(self.pid_params, self.port, self.baud, self.timeout)
        time.sleep(1)
        self.mySerializer.connect()
        time.sleep(1)
        
        # Create and start the base controller.
        if self.pid_params['use_base_controller']:
            rospy.loginfo("Starting Serialzier base controller...")
            self.base_controller = base_controller(self.mySerializer, "Serializer PID")
            self.base_controller.start()
            
        self.sensors = dict({})
        
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
                    time.sleep(0.01)
                        
                for sensor, params in self.digital_sensors.iteritems():
                    if params['type'] == "Ping":
                        self.sensors[sensor] = self.mySerializer.get_Ping(params['pin'])
                    else:
                        self.sensors[sensor] = self.mySerializer.get_io(params['pin'])
                    time.sleep(0.01)

                self.msg = SensorState()
                self.msg.name = list()
                self.msg.value = list()
                           
                for sensor, value in self.sensors.iteritems():
                    self.msg.name.append(sensor)
                    try:
                        self.msg.value.append(round(float(value), 1))
                    except:
                        try:
                            self.msg.value.append(float(value))
                        except:
                            self.msg.value.append(-999.0)
               
                self.msg.header.frame_id = "sensors"
                self.msg.header.stamp = rospy.Time.now()     
                self.sensorStatePub.publish(self.msg)
                rosRate.sleep()
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
        return GetAnalogResponse(self.mySerializer.get_analog(req.pin))
    
    def PingHandler(self, req):
        try:
            sonar = self.mySerializer.get_Ping(req.pin, req.cached)
        except:
            rospy.logerror("Bad sonar value on pine " + str(req.pin) + ". Value was: " + str(sonar))
        return PingResponse(sonar)

    def GP2D12Handler(self, req):
        return GP2D12Response(self.mySerializer.get_GP2D12(req.pin, req.cached))
    
    def VoltageHandler(self, req):
        return VoltageResponse(self.mySerializer.voltage(req.cached))

    def PhidgetsTemperatureHandler(self, req):
        return PhidgetsTemperatureResponse(self.mySerializer.get_PhidgetsTemperature(req.pin, req.cached))
    
    def PhidgetsVoltageHandler(self, req):
        return PhidgetsVoltageResponse(self.mySerializer.get_PhidgetsVoltage(req.pin, req.cached))
    
    def PhidgetsCurrentHandler(self, req):
        return PhidgetsCurrentResponse(self.mySerializer.get_PhidgetsCurrent(req.pin, req.cached))
        
    def shutdown(self):
        rospy.loginfo("Shutting down Serializer Node.")
        
if __name__ == '__main__':
    try:
        mySerializer = SerializerROS()
    except rospy.ROSInterruptException:
        pass
