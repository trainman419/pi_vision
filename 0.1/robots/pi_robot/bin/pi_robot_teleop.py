#!/usr/bin/env python

"""
    Driving Pi Robot with the keyboard
    
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
import time
from geometry_msgs.msg import Twist

from Tkinter import *

rospy.init_node("pi_robot")
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist)
base_cmd = Twist()
linear_speed = 0.0
angular_speed = 0.0

root = Tk()
prompt = '      Press the arrow keys to move Pi.  Press the space bar to stop.      '
label1 = Label(root, text=prompt, width=len(prompt), height=20)
label1.pack()

base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0

def key(event):
                
    if event.char == event.keysym:
        msg = 'Not a valid movement command %r' % event.char
    elif len(event.char) == 1:
        if event.keysym == "space":
            msg =  "Stopping!"
            base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0
        else:
            msg = 'Not a valid movement command %r (%r)' % (event.keysym, event.char)
    else:
        if event.keysym == "Up":
            msg = "Forward"
            base_cmd.angular.z = 0
            base_cmd.linear.x += 0.05
        elif event.keysym == "Right":
            msg = "Right"
            #base_cmd.linear.x = 0.0
            base_cmd.angular.z -= 0.1
        elif event.keysym == "Left":
            msg = "Left"
            #base_cmd.linear.x = 0.0
            base_cmd.angular.z += 0.1
        elif event.keysym == "Down":
            msg = "Back"
            base_cmd.angular.z = 0
            #base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0
            #cmd_vel_pub.publish(base_cmd)
            base_cmd.linear.x -= 0.05
        else:
            msg = 'Not a valid movement command %r' % event.keysym
    
    cmd_vel_pub.publish(base_cmd)
    
    label1.config(text=msg)

def keyboard_drive():
    root.bind_all('<Key>', key)
    root.mainloop()

def test_drive():
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0
    while not rospy.is_shutdown():
        base_cmd.angular.z = 0.2
        cmd_vel_pub.publish(base_cmd)
        time.sleep(1)
        base_cmd.angular.z = -0.2
        cmd_vel_pub.publish(base_cmd)
        time.sleep(1)
        base_cmd.angular.z = 0.0
        cmd_vel_pub.publish(base_cmd)
        time.sleep(1)


if __name__ == '__main__':
    try:
        keyboard_drive()
        #test_drive()
    except rospy.ROSInterruptException:
        base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0
        cmd_vel_pub.publish(base_cmd)
