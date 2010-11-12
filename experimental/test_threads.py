"""
    Thread test: Accessing the Serializer using multiple threads.
    
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

import serializer_driver as SerializerAPI
import threading, time, os
from datetime import datetime
import random

if os.name == "posix":
    portName = "/dev/ttyUSB0" # Change this to your main Serializer port!
    #portName = "/dev/rfcomm0" # For bluetooth on Linux
    # Note: On Linux, after connecting to the Bluetooth adapter, run the command
    # sudo rfcomm bind /dev/rfcomm0
else:
    portName = "COM21" # Change this to your main Serializer port!
    
baudRate = 57600 # Change this to your Serializer baud rate!

mySerializer = SerializerAPI.Serializer(port=portName, baudrate=baudRate, timeout=0.05)
mySerializer.connect()

class Thread1(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.finished = threading.Event()
        self.interval = 0.05 # Access the sensors 20 times per second.
        self.daemon = False
        self.count = 0

    def run(self):
        max_delay = 0
        while not self.finished.isSet():
            start = datetime.now()
            #output = "Thread 1 Voltage: " + str(mySerializer.voltage()) + " IR: " +  str(mySerializer.get_GP2D12(4)) + "Sonar: " +  str(mySerializer.get_Ping(4))
            output = "Encoders: " + str(mySerializer.get_encoder_count([1, 2]))
            print output
            delay = (datetime.now() - start).microseconds / 1000
            #output = "Thread 1:", delay
            #print output
            if delay > max_delay:
                max_delay = delay
                print "MAX:", max_delay
            time.sleep(self.interval)
            
    def stop(self):
        print "Stopping Node 1 Thread ...",
        self.finished.set()
        self.join()
        print "Done."
            
class Thread2(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.finished = threading.Event()
        self.interval = 0.5  # Access the voltage 2 times per second.
        self.daemon = False
        self.count = 0

    def run(self):
        max_delay = 0
        while not self.finished.isSet():
            start = datetime.now()
            output = "Thread 2:", mySerializer.rotate(1.57, 0.2)
            while mySerializer.get_pids():
                time.sleep(0.05)
            output = "Thread 2:", mySerializer.rotate(-1.57, 0.2)
            while mySerializer.get_pids():
                time.sleep(0.05)
            #time.sleep(2)
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

#thread1 = Thread1()
thread2 = Thread2()

#thread1.start()
thread2.start()

time.sleep(600)

#thread1.stop()
thread2.stop()

mySerializer.stop()
mySerializer.close()

