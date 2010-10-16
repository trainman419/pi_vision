"""
    Example 1: Read values from a number of analog and digital sensors and control
    one or more servos.
    
    The Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2010 Patrick Goebel.  All right reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
    NOTE: See the offical SerializerTM manual at:
    http://www.roboticsconnection.com/multimedia/docs/Serializer_3.0_UserGuide.pdf
"""

import serializer as Serializer
import time, os

def SharpGP2Y0A0(reading):
    range = 7140.66 * pow(reading + 1, -0.87507) - 10
    return range
    
if os.name == "posix":
    portName = "/dev/ttyUSB0"   # Change this to your main Serializer port!   
    #portName = "/dev/rfcomm0" # For bluetooth on Linux
    # Note: On Linux, after connecting to the Bluetooth adapter, run the command
    # sudo rfcomm bind /dev/rfcomm0
else:
    portName = "COM43" # Change this to your main Serializer port!
    
baudRate = 57600 # Change this to your Serializer baud rate!

mySerializer = Serializer.Serializer(port=portName, baudrate=baudRate, timeout=0.5)
mySerializer.connect()
    
print "Firmware Version", mySerializer.fw()
print "Units", mySerializer.get_units()
print "Baudrate", mySerializer.get_baud()
    
""" The following two lines assume we have a Ping sonar sensor attached to
    GPIO pin 4 and a Sharp GP2D12 IR sensor to anlog pin 4.
"""

myPing = Serializer.Ping(mySerializer, 4)
myIR = Serializer.GP2D12(mySerializer, 4)

print "Moving servo on GPIO pin 9 (servo ID 2)", mySerializer.servo(2, -80)
time.sleep(1.5)

count = 0
clock = 0
data = list()
times = list()
mySerializer.servo(2, 60)
start = time.clock()
for i in range(30):

    #print "All Analog Sensor Values:", mySerializer.get_all_analog()
    #print "Analog values from the cache:", mySerializer.analog_sensor_cache
    #print "Serializer voltage from the cache", mySerializer.voltage(cached=True)
    #print "Ping Sonar reading on digital pin 4:", mySerializer.pping(4)
    #print "Ping reading using the Ping class and reading from the cache:", myPing.value(cached=True)
    #print "Sharp IR reading using the Sharp class on analog pin 4:", myIR.value()
    #print "Sharp IR reading from cache:", myIR.value(cached=True)
    
    data.append(SharpGP2Y0A0(mySerializer.sensor(3)))
    #delta = time.clock() - start
    #times.append(delta)
    #count += 1
    #clock += delta

#print "Ave Time:", clock / count
print "Time:", time.clock() - start
print data
#print times
print "\nTesting completed, shutting down."

mySerializer.stop()
mySerializer.close()

