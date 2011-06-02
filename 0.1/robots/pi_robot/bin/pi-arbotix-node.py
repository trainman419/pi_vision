#!/usr/bin/env python

"""
  ArbotiX ROS Node: serial connection to an ArbotiX board w/ PyPose/NUKE/ROS
  Copyright (c) 2008-2010 Michael E. Ferguson.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import roslib; roslib.load_manifest('arbotix')
import rospy

from sensor_msgs.msg import JointState
from arbotix.msg import *

from arbotix.arbotix import ArbotiX # does this look ridiculous to anyone else?
from arbotix.srv import *
from arbotix.ax12 import *

# TODO: generalize these, add init.py in packages
from arbotix_sensors.pml import *
from arbotix_sensors.v_monitor import *
from arbotix_controllers.base_controller import *
from arbotix_controllers.nuke_controller import *
from arbotix_controllers.joint_controller import *
#from arbotix_controllers.joint_traj_controller import *

from math import sin,cos,pi,radians
from datetime import datetime

###############################################################################
# Servo handling classes    
class DynamixelServo():
    """ Class to handle services and updates for a single Dynamixel Servo, on 
        an ArbotiX robocontroller's AX/RX-bus. """
    def __init__(self, name, params, device, single=False):
        self.name = name
        self.device = device                        # ArbotiX instance

        self.id = -1
        self.neutral = 512                          # adjust for EX-106, etc
        self.ticks = 1024                           # adjust for EX-106, etc
        self.rad_per_tick = radians(300.0)/1024     # adjust for EX-106, etc
        self.max_angle = radians(150)               # limit angle, radians
        self.min_angle = radians(-150)
        self.max_speed = 11.0                       # speed in radians per second
        self.max_speed_ax12 = radians(684.0)        # From the AX-12+ manual: max speed = 114 rpm - 684 deg/s
        self.speed_rad_per_tick = self.max_speed_ax12 / 1024       
        self.invert = False
        self.sync = True
        self.setParams(params)

        self.angle = 0.0                            # current position
        self.velocity = 0.0

        # some callbacks
        if single:
            self.srvRead = rospy.Service(self.name+'_getangle',GetAngle, self.getAngleCb)
            self.srvMoving = rospy.Service(self.name+'_ismoving',IsMoving, self.isMovingCb)
            self.srvWrite = rospy.Service(self.name+'_setangle',SetAngle, self.setAngleCb)

        # this will get indented once SetJoints stuff works
        self.srvRelax = rospy.Service(self.name+'_relax',Relax, self.relaxCb)
        #self.srvMoving = rospy.Service(self.name+'_moving',IsMoving, self.isMovingCb)


    def setParams(self, params):
        for key in params.keys():
            if key=='invert':
                if int(params[key]) > 0:
                    self.invert = True
            elif key=='sync':
                self.sync = params[key]
            elif key=='max_speed':
                self.max_speed = float(params[key])
            elif key=='max_angle':
                self.max_angle = radians(float(params[key]))
            elif key=='min_angle':
                self.min_angle = radians(float(params[key]))
            elif key=='ticks':
                self.ticks = int(params[key])
            elif key=='range':
                self.rad_per_tick = radians(float(params[key]))/self.ticks
            elif key=='neutral':
                self.neutral = params[key]
            elif key=='id':
                self.id = int(params[key])
            else:
                rospy.logerr("Parameter '" + key + "' not recognized.")

    def setAngleCb(self, req):
        self.setAngle(req.angle)
        return SetAngleResponse()
    
    def setVelocityCb(self, req):
        self.setVelocity(req.speed)
        return SetVelocityResponse()

    def getAngleCb( self, req ):
        """ ROS service to get angle of servo (in radians) """
        return GetAngleResponse( self.getAngle() )

    def isMovingCb(self, req):
        return IsMovingResponse( self.isMoving() )

    def relaxCb(self, req):
        """ Turn off servo torque, so that it is pose-able. """
        self.device.disableTorque(self.id)
        return RelaxResponse()

    def setAngle(self, ang):
        if ang > self.max_angle or ang < self.min_angle:
            rospy.logerr("Servo "+self.name+": angle out of range ("+str(ang)+"). Limits are: " + str(self.min_angle) + "," + str(self.max_angle))  
            return 
        self.angle = ang    # store it for joint state updates
        if self.invert:
            ang = ang * -1.0
        ticks = int(round( ang / self.rad_per_tick ))
        ticks += int(self.neutral)
        self.device.setPosition(self.id, ticks)
        
    def setVelocity(self, vel):
        if vel > self.max_speed:
            rospy.loginfo("Servo "+self.name+": speed out of range ("+str(vel)+"). Max speed is: " + str(self.max_speed))  
            vel = self.max_speed 
        self.velocity = vel    # store it for joint state updates
        ticks = max(1, int(round( vel / self.speed_rad_per_tick ))) # A value of 0 = "fast as you can" so avoid this
        self.device.setSpeed(self.id, ticks)
        
    def getAngle(self, pos=None):
        """ Find angle in radians by reading from servo, or
            by using pos passed in from a sync read.  """
        if pos == None:
            pos = self.device.getPosition(self.id)
        if pos != -1:
            angle = (pos - self.neutral) * self.rad_per_tick
            # Check for erroroneous values.
            if abs(angle) > (2.0 * pi):     
                return self.angle
            if self.invert:
                angle = angle * -1.0
            self.angle = angle
        return self.angle
    
    def getVelocity(self, spd=None):
        """ Find speed in radians/sec by reading from servo, or
            by using spd passed in from a sync read.  """
        if spd == None:
            spd = self.device.getSpeed(self.id)
        if spd != -1:
            velocity = spd *  self.speed_rad_per_tick
            # Check for erroroneous values.
            if abs(velocity) >= self.max_speed_ax12:     
                return self.velocity
            if self.invert:
                velocity = velocity * -1.0
            self.velocity = velocity
        return self.velocity
    
    def isMoving(self):
        return int(self.device.read(self.id, P_MOVING, 1)[0])

    def getAngleStored(self):
        return self.angle
    
    def getVelocityStored(self):
        return self.velocity

class HobbyServo(DynamixelServo):
    """ Class to handle services and updates for a single Hobby Servo, connected to 
        an ArbotiX robocontroller. A stripped down version of the DynamixelServo. """
    def __init__(self, name, params, device, single=False):
        self.name = name
        self.device = device                        # ArbotiX instance

        self.id = -1
        self.neutral = 1500                         # might be adjusted for crappy servos
        self.ticks = 2000
        self.rad_per_tick = radians(180.0)/2000     # 180 degrees over 500-2500ms 
        self.max_angle = radians(90)                # limit angle, radians
        self.min_angle = radians(-90)
        self.invert = False
        self.setParams(params)

        self.angle = 0.0                            # current position

        # a callback
        if single: 
            self.srvWrite = rospy.Service(self.name+'_setangle',SetAngle, self.setAngleCb)

    def setAngleCb(self, req):
        """ Callback to set position to angle, in radians. """
        ang = req.angle
        if ang > self.max_angle or ang < self.min_angle:
            rospy.logerr("Servo "+self.name+": angle out of range ("+str(ang)+")")            
            return SetAngleResponse()
        self.angle = ang    # store it for joint state updates
        if self.invert:
            ang = ang * -1.0
        ticks = int(round( ang / self.rad_per_tick ))
        ticks += self.neutral
        rospy.loginfo("Servo "+self.name+": set to "+str(ticks))
        self.device.setServo(self.id, ticks)
        return SetAngleResponse()        

    def getAngle(self):
        """ Find angle in radians """
        return self.angle

class ArbotiX_ROS(ArbotiX):
    
    def __init__(self):
        #rospy.init_node('arbotix', log_level=rospy.DEBUG)
        rospy.init_node('arbotix')
        # load configurations    
        port = rospy.get_param("~port", "/dev/ttyUSB0")                     
        baud = int(rospy.get_param("~baud", "38400"))

        # start an arbotix driver
        ArbotiX.__init__(self, port, baud)        
        rospy.loginfo("Started ArbotiX-ROS on port "+port)

        # initialize servos and state publishing
        use_sync = rospy.get_param("~use_sync",True)                        # use sync read?
        use_single = rospy.get_param("~use_single_services",False)          # use single read/write services?
        dynamixels = rospy.get_param("~dynamixels", dict())
        # complete list of servos being controlled, index = servo name, value = servo object
        self.servos = dict()                                                
        self.sync_servos = list()   # ids of servos we will sync_read
        self.sync_names = list()    # names of servos we will sync_read (in same order)
        self.no_sync_names = list() # names of servos we can't sync read
        
        for name in dynamixels.keys():
            servo = DynamixelServo(name, dynamixels[name], self, use_single) 
            self.servos[servo.name] = servo 
            if servo.sync: 
                self.sync_servos.append(servo.id)    
                self.sync_names.append(servo.name)    
            else:
                self.no_sync_names.append(servo.name)
        if len(self.sync_servos) == 0:
            use_sync = False

        servos = rospy.get_param("~servos", dict())
        for name in servos.keys():
            servo = HobbyServo(name, servos[name], self)
            self.servos[servo.name] = servo
            self.no_sync_names.append(servo.name)

        self.jointStatePub = rospy.Publisher('joint_states', JointState)
        self.extraJointStatePub = rospy.Publisher('extra_joint_states', ExtraJointState)

        # initialize digital/analog IO
        rospy.Service('GetDigital',GetDigital,self.getDigitalCb)   
        rospy.Service('GetAnalog',GetAnalog,self.getAnalogCb)
        rospy.Service('SetDigital',SetDigital,self.setDigitalCb)

        # initialize controllers
        controller_list = rospy.get_param("~controllers",list())
        if len(controller_list) == 0:
            # launch a default controller
            joints = joint_controller(self, "joint_controller", self.servos.keys())
        else:
            for controller, params in controller_list.items():
                if params["type"] == "joint_controller":
                    jc = joint_controller(self, controller)
                    jc.start()
                #elif params["type"] == "joint_traj_controller":
                #    jtc = joint_traj_controller(self, controller)
                #    jtc.start()
                elif params["type"] == "base_controller":
                    bc = base_controller(self, controller)
                    bc.start()
        # initialize sensors
        sensor_list = rospy.get_param("~sensors",list())
        if len(sensor_list) > 0:
            for sensor, params in sensor_list.items():
                if params["type"] == "pml":
                    mylidar = pml(self, sensor)
                    mylidar.start()
                if params["type"] == "v_monitor":
                    vmon = v_monitor(self, sensor)
                    vmon.start()

        # publish joint states (everything else is a service/topic callback)
        r = rospy.Rate(int(rospy.get_param("~rate",10)))
        
        while not rospy.is_shutdown():
                
            # publish joint states
            msg = JointState()
            msg.name = list()
            msg.position = list()
            msg.velocity = list()
            msg.effort = list()
            
            extra_msg = ExtraJointState()
            extra_msg.moving = list()

            try:
                # TODO: add torque/heat recovery
                #   a.write(id,P_TORQUE_LIMIT_L,[255,3])
                if use_sync: 
                    # arbotix/servostik/wifi board sync_read
                    val_pos = self.syncRead(self.sync_servos, P_PRESENT_POSITION_L, 2)
                    val_spd = self.syncRead(self.sync_servos, P_PRESENT_SPEED_L, 2)

                    if val_pos != None:            
                        i = 0        
                        for name in self.sync_names:
                            msg.name.append(name)
                            msg.position.append(self.servos[name].getAngle( val_pos[i]+(val_pos[i+1]<<8) ))
                            msg.velocity.append(self.servos[name].getVelocity( val_spd[i]+(val_spd[i+1]<<8) ))

                            extra_msg.moving.append(self.servos[name].isMoving())
                            i = i + 2
                        for name in self.no_sync_names: 
                            msg.name.append(name)
                            msg.position.append(self.servos[name].getAngleStored())
                            msg.velocity.append(self.servos[name].getVelocityStored())
                            extra_msg.moving.append(self.servos[name].isMoving())
                else:
                    # direct connection, or other hardware with no sync_read capability
                    for name in self.sync_names:
                        msg.name.append(name)
                        msg.position.append(self.servos[name].getAngle())
                        msg.velocity.append(self.servos[name].getVelocity())

                        extra_msg.moving.append(self.servos[name].isMoving())
                    for name in self.no_sync_names: 
                        msg.name.append(name)
                        msg.position.append(self.servos[name].getAngleStored())
                        msg.velocity.append(self.servos[name].getVelocityStored())
                        extra_msg.moving.append(self.servos[name].isMoving())
            except:
                rospy.loginfo("Error in filling joint_states message")
    
            now = rospy.Time.now()
            msg.header.stamp = now
            self.jointStatePub.publish(msg)
            extra_msg.header.stamp = now
            self.extraJointStatePub.publish(extra_msg)    
            r.sleep()

    def getDigitalCb(self, req):
        return GetDigitalResponse( self.getDigital(req.pin) )

    def getAnalogCb(self, req):    
        return GetAnalogResponse( self.getAnalog(req.pin) )

    def setDigitalCb(self, req):
        self.setDigital(req.pin, req.value, req.dir)
        return SetDigitalResponse()                        

if __name__ == "__main__":
    a = ArbotiX_ROS()

