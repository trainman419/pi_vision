"""
    A Python library for the Robotics Connection SerializerTM micro controller.
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
    
    http://www.gnu.org/licenses/gpl.html
      
    NOTE: See the offical SerializerTM manual at:
    http://www.roboticsconnection.com/multimedia/docs/Serializer_3.0_UserGuide.pdf
        
    Basic Usage:

    mySerializer = Serializer(port="COM12", baudrate=19200, timeout=5)
    mySerializer.connect()
    myPing = Ping(mySerializer, 4)  
    myIR = GP2D12(mySerializer, 4)
    
    print myPing.value()
    print myIR.value()
    
    mySerializer.stop()
    mySerializer.close()
    
    See the example files for more details.
"""

import serial
import threading
import math
import os
from serial.serialutil import SerialException

class Serializer():  
    ''' Configuration Parameters
    '''    
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12
    UNITS = 0                   # 1 is inches, 0 is metric (cm for sensors, meters for wheels measurements) and 2 is "raw"
    WHEEL_DIAMETER = 0.127      # meters (5.0 inches) meters or inches depending on UNITS
    WHEEL_TRACK = 0.325         # meters (12.8 inches) meters or inches units depending on UNITS
    ENCODER_RESOLUTION = 624    # encoder ticks per revolution of the wheel without external gears
    GEAR_REDUCTION = 1.667      # This is for external gearing if you have any.

    VPID_P = 2  # Proportional
    VPID_I = 0  # Integral
    VPID_D = 5  # Derivative                                                                               
    VPID_L = 45 # Loop: this together with UNITS and WHEEL_DIAMETER determines real-world velocity
    
    DPID_P = 1  # Proportional
    DPID_I = 0  # Integral
    DPID_D = 0  # Derivative 
    DPID_A = 5  # Acceleration
    DPID_B = 5  # Dead band
    
    MILLISECONDS_PER_PID_LOOP = 1.6 # Do not change this!  It is a fixed property of the Serializer PID controller.
    LOOP_INTERVAL = VPID_L * MILLISECONDS_PER_PID_LOOP / 1000 # in seconds
    
    INIT_PID = False # Set to True if you want to update UNITS, VPID and DPID parameters.  Otherwise, those stored in the Serializer's firmware are used.**
    
    def __init__(self, port="/dev/ttyUSB0", baudrate=19200, timeout=5): 
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.wheel_diameter = self.WHEEL_DIAMETER
        self.wheel_track = self.WHEEL_TRACK
        self.encoder_resolution = self.ENCODER_RESOLUTION
        self.gear_reduction = self.GEAR_REDUCTION
        self.loop_interval = None
        self.ticks_per_meter = None
        self.messageLock = threading.Lock()
            
        ''' An array to cache analog sensor readings'''
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        
        ''' An array to cache digital sensor readings'''
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS
    
    def connect(self):
        try:
            print "Connecting to Serializer on port", self.port, "..."
            self.port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout, writeTimeout=self.timeout)
            if self.get_baud() != self.baudrate:
                raise SerialException
            print "Connected at", self.baudrate, "baud."
            
            # Take care of the UNITS, VPID and DPID parameters for PID drive control.
            if self.INIT_PID:
                self.init_PID()
            else:
                self.units = self.get_units()
                [self.VPID_P, self.VPID_I, self.VPID_D, self.VPID_L] = self.get_vpid()
                [self.DPID_P, self.DPID_I, self.DPID_D, self.DPID_A, self.DPID_B] = self.get_dpid()
                
            if self.units == 0:
                self.ticks_per_meter = int(self.encoder_resolution / (self.wheel_diameter * math.pi))
            elif self.units == 1:
                self.ticks_per_meter = int(self.encoder_resolution / (self.wheel_diameter * math.pi * 2.54 / 100.0))
                    
            self.loop_interval = self.VPID_L * self.MILLISECONDS_PER_PID_LOOP / 1000

        except SerialException:
            print "Cannot connect to Serializer!"
            print "Make sure you are plugged in and turned on."
            os._exit(1)
            
    def init_PID(self):
        print "Updating Units and PID parameters."
        self.set_units(self.UNITS)
        self.set_vpid(self.VPID_P, self.VPID_I, self.VPID_D, self.VPID_L)
        self.set_dpid(self.DPID_P, self.DPID_I, self.DPID_D, self.DPID_A, self.DPID_B)

    def open(self): 
        ''' Open the serial port.
        '''
        with self.messageLock:
            self.port.open()
            print self.recv()

    def close(self): 
        ''' Close the serial port.
        '''
        with self.messageLock:
            self.port.close() 
    
    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        self.port.write(cmd + '\r')

    def recv(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        return (self.port.readline(eol='>')[0:-3]).strip()
            
    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv()
        return ack == 'ACK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        msg = self.recv()
        try:
            return int(msg)
        except:
            return msg

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            return map(int, self.recv().split())
        except:
            return []

    def execute(self, cmd):
        ''' Thread safe execution of "cmd" on the SerializerTM returning a single value.
        '''
        with self.messageLock:
            try:
                self.port.flushInput()
            except:
                pass
            self.port.write(cmd + '\r')
            return (self.port.readline(eol='>')[0:-3]).strip()

    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the SerializerTM returning an array.
        '''
        with self.messageLock:
            try:
                self.port.flushInput()
            except:
                pass
            self.port.write(cmd + '\r')
            try:
                return map(int, self.recv().split())
            except:
                return []
        
    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the SerializerTM returning True if response is ACK.
        '''
        with self.messageLock:
            try:
                self.port.flushInput()
            except:
                pass
            self.port.write(cmd + '\r')
            ack = self.recv()
            return ack == 'ACK'
        
    def execute_int(self, cmd):
        ''' Thread safe execution of "cmd" on the SerializerTM returning an int.
        '''
        with self.messageLock:
            try:
                self.port.flushInput()
            except:
                pass
            self.port.write(cmd + '\r')
            return self.recv_int()

    def fw(self):
        ''' The fw command returns the current firmware version.
        '''
        return self.execute('fw')

    def reset(self):
        ''' The reset command resets the SerializerTM board and reboots it. You
            will see the SerializerTM welcome screen appear after a short delay.
            Once the welcome string appears, the SerializerTM is ready to accept
            commands.
        '''
        return self.execute('reset')

    def blink_led(self, id, rate):
        ''' Usage 1: blink_led(id, rate)  
            Usage 2: blink_led([id1, id2], [rate1, rate2]) 
            The blink_led command can blink one of the two onboard green LEDs
            simultaneously, or individually. Each complex parameter is comprised
            of an <ledId:blinkRate> pair. The ledId specifies which of the two
            green LEDs to blink, and blinkRate specifies the delay between blinks.
            The minimum blink rate is 1, and the largest is 127. A value of 0 turns
            the led off.
        '''
        if type(id) == int: id = [id]
        if type(rate) == int: rate = [rate]          
        return self.execute_ack('blink %s' %' '.join(map(lambda x: '%d:%d' %x, zip(id,rate))))

    def get_compass(self, i2c_addr=None):
        ''' Usage 1: heading = get_compass()
            Usage 2: heading = get_compass(i2c_addr)           
            The get_compass command queries a Devantech CMPS03 Electronic
            compass module attached to the Serializers I2C port.
            The current heading is returned in Binary Radians, or BRADS.To
            convert BRADS to DEGREES, multiply BRADS by 360/255 (~1.41).
            The default I2C address is 0xC0, however another I2C address can be
            supplied as an optional parameter.
        '''
        return 360. / 255. * self.execute_int('cmps03 %X' %i2c_addr if i2c_addr else 'cmps03' )

    def get_encoder(self):
        ''' The get_encoder command returns the encoder type for the SerializerTM,
            single (0) or quadrature (1).
        '''
        return self.execute_int('cfg enc');

    def set_encoder(self, encoder_type):
        ''' The set_encoder command configures the internal encoder type to be either
            single (0) or quadrature (1) type. This information is saved in the
            EEPROM, so that the configuration will be retained after a reboot. If
            you are using a quadrature encoder (dual channels), and the Serializer
            is configured for single encoder operation, then the second quadrature
            channel will be ignored. Thus make sure the correct encoder type is
            configured according to your setup. The cfg enc command without a
            parameter returns the value currently stored in EEPROM.
        '''
        return self.execute_ack('cfg enc %d' %encoder_type);

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return self.execute_int('cfg baud');

    def set_baud(self, baudrate):
        ''' The set_baud command configures the serial baud rate on the
            SerializerTM. Values can be 0=2400, 1=4800, 2=9600, 3=19200,
            4=57600, or 5=115200. You can also type in the actual baud rate
            string as well (e.g. 19200). The default baud rate used to
            communicate with the Serializer is 19200. The cfg baud command
            without a parameter returns the value currently stored in EEPROM.
        '''
        return self.execute_ack('cfg baud %d' %baudrate);

    def get_units(self):
        ''' The get_units returns the current units used for sensor
            readings. Values are 0 for metric mode, 1 for English mode, and 2 for
            raw mode.  In raw mode, srf04, srf05, pping, and maxez1 return
            reading in units of 0.4us. srf08 and srf10 return readings of 1us..
        '''
        return int(self.execute('cfg units'))

    def set_units(self, units):
        ''' The set_units command sets the internal units used for sensor
            readings. Values are 0 for metric mode, 1 for English mode, and 2 for
            raw mode.  In raw mode, srf04, srf05, pping, and maxez1 return
            reading in units of 0.4us. srf08 and srf10 return readings of 1us.
            The cfg units command without a parameter returns the value
            currently stored in EEPROM.
        '''
        self.units = units
        return self.execute_int('cfg units %d' %units);

    def get_encoder_count(self, id):
        ''' Usage 1: get_encoder_count(id)
            Usage 2: get_encoder_count([id1, id2])
            The get_encoder_count command returns the values of the encoder
            count (channel B) for the specified encoder Id(s). NOTE: The encoder
            counts for channel A are used for internal VPID and DPID algorithms.
        '''
        if type(id) == int: id=[id]
        return self.execute_array('getenc %s' %' '.join(map(str, id)))

    def clear_encoder(self, id):
        ''' Usage 1: clear_encoder(id)
            Usage 2: clear_encoder([id1, id2])
            The clear_encoder command clears the values of the encoder
            count (channel B) for the specified encoder Id.
        '''
        if type(id) == int: id=[id]
        return self.execute_ack('clrenc %s' %' '.join(map(str, id)))

    def get_io(self, id):
        ''' Usage 1: get_io(id)
            Usage 2: get_io([id1, id2, id3, ... , idN]) 
            The get_io command changes the pin, pinId (range 0-12), to an input
            (if it was an output), and gets the value of the specified General
            Purpose I/O lines on the SerializerTM. The valid range of I/O pin Ids is
            0 thru 12. More than one pid can be specified by enter a list as argument.
        '''
        if type(id) == int: id=[id]
        values = self.execute_array('getio %s' %' '.join(map(str, id)))
        n = len(id)
        for i in range(n):
            self.digital_sensor_cache[id[i]] = values[i]
        if n == 1:
            return values[0]
        else:
            return values

    def set_io(self, id, val):
        ''' Usage 1: set_io(id, val)
            Usage 2: set_io([id1, id2, ... , idN], [val1, val2, ..., valN)  
            The set_io command sets the specified General Purpose I/O line pinId
            (range 0-12) on the SerializerTM to the specified value. Each complex
            parameter is a <pinId:value> pair, where the valid range of pinId is 0
            thru 12, and value can be 0 or 1 which corresponds to 0v or +5V
            respectively.  Also I/O lines 4,5,6,7, 8 and 9 cannot be used if you
            have servos connected to them. Pin 10, 11, and 12 correspond to the
            internal h-bridge enable, SCL, and SDA respectively.
        '''
        if type(id) == int: id = [id]
        if type(val) == int: val = [val]          
        return self.execute_ack('setio %s' %' '.join(map(lambda x: '%d:%d' %x, zip(id, val))))

    def get_maxez1(self, triggerPin, outputPin):
        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
            sensor connected to the General Purpose I/O lines, triggerPin, and
            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
            power up, otherwise it wont range correctly for object less than 6
            inches away! The sensor reading defaults to use English units
            (inches). The sensor distance resolution is integer based. Also, the
            maxsonar trigger pin is RX, and the echo pin is PW.
        '''
        return self.execute_int('maxez1 %d %d' %(triggerPin, outputPin))

    def stop(self):
        ''' Stop both motors.
        '''
        return self.execute('stop')

    def get_vpid(self):
        ''' Get the PIDL parameter values.
        '''       
        pidl = self.execute('vpid')
        return map(lambda x: int(x.split(':')[1]), pidl.split())

    def set_vpid(self, prop, integ, deriv, loop):
        ''' The set_vpid command sets the PIDL (Proportional, Integral,
            Derivative, and Loop) parameters for the Velocity PID control on the
            SerializerTM. The PIDL parameters are parsed, and saved (in
            eeprom). For more information on PIDL control, see the PIDL
            configuration section below.  By default the Serializer VPID
            parameters are configured to work with our Traxster Robot Kit
        '''
        [self.VPID_P, self.VPID_I, self.VPID_D, self.VPID_L] = [prop, integ, deriv, loop]
        return self.execute_ack('vpid %d:%d:%d:%d' %(prop, integ, deriv, loop))
    
    def digo(self, id, dist, vel):
        ''' Usage 1: m(id, dist, vel)
            Usage 2: digo([id1, id2], [dist1, dist2], [vel1, vel2]) 
            Simply put, the digo command allows you to command your robot to
            travel a specified distance, at a specified speed. This command uses
            the internal VPID and DPID algorithms to control velocity and distance.
            Therefore, you must have dual motors, and dual wheel encoders
            connected to the Serializer motor ports and encoder inputs.
        '''
        if type(id) == int: id = [id]
        if type(dist) == int: id = [dist]
        if type(vel) == int: vel = [vel]          
        return self.execute('digo %s' %' '.join(map(lambda x: '%d:%d:%d' %x, zip(id, dist, vel))))
    
    def get_dpid(self):
        ''' Get the PIDA parameter values.
        '''  
        dpid = self.execute('dpid')
        return map(lambda x: int(x.split(':')[1]), dpid.split())

    def set_dpid(self, prop, integ, deriv, accel, dead_band):
        ''' The set_dpid command gets/sets the PIDA (Proportional, Integral,
            Derivative, and Acceleration) parameters for the distance PID control
            on the SerializerTM. If the PIDA parameters are absent, the PIDA
            values are returned. Otherwise the PIDA parameters are parsed, and
            saved (in eeprom).  
        '''
        [self.DPID_P, self.DPID_I, self.DPID_D, self.DPID_A, self.DPID_B] = [prop, integ, deriv, accel, dead_band]
        return self.execute_ack('dpid %d:%d:%d:%d:%d' %(prop, integ, deriv, accel, dead_band))
     
    def set_rpid(self, r):
        ''' The rpid command sets the default PID params known to work with
            either the Stinger or Traxster Robotic Kits in the firmware. This makes
            it quick and easy to set up the PID params for both robots.
        '''
        return self.execute_ack('rpid %c' %r)

    def get_pids(self):
        ''' Once a digo command is issued, an internal state variable within the
            firmware is set to 1, and it stays in that state until the algorithm has
            completed. Upon completion, the state is set to 0. The pids
            command simply returns the value of the internal variable to
            determine if the algorithms is currently busy, or if it has finished, thus
            allowing subsequent digo commands to be issued w/o clobbering
            previous ones.
        '''
        return self.execute_int('pids');
    
    def pwm(self, id, vel, rate=None):
        ''' Usage 1: pwm(id, vel)
            Usage 2: pwm(id, vel, rate=r)
            Usage 3: pwm([id1, id2], [vel1, vel2]) 
            Usage 4: pwm([id1, id2], [vel1, vel2], rate=r)   
            The pwm command sets the Pulse Width Modulation value for
            Motor 1 & Motor 2. Each complex parameter is a motor
            <motorId:pwm value> pair, where the motor id can be 1 or 2, and the
            pwm value can be -100 to 100. Each complex parameter pair is
            separated by one or more spaces.
            The optional rate parameter allows the
            motor(s) speed(s)s to be ramped up or down to the specified speed
            from the current motor speed.
        '''
        if type(id) == int: id = [id]
        if type(vel) == int: vel = [vel]          
        if rate != None:
            cmd = 'pwm r:%d ' %rate + ' '.join(map(lambda x: '%d:%d' %x, zip(id, vel)))
        else:         
            cmd = 'pwm %s' % ' '.join(map(lambda x: '%d:%d' %x, zip(id, vel)))
        return self.execute(cmd)

    def step(self, dir, speed ,steps):
        ''' The step command is used to step a bipolar stepper motor in direction
            dir, at the specified speed, for the specified number of steps.
            The dir parameter specifies a CW or CCW rotational direction, and its
            value can be either 0 (CCW) or 1(CW). Your specific direction is based
            on the way that you have your bipolar motor connected to the
            Serializer.  The speed parameter can be a value from 0 to 100.
            The steps parameter specifies the maximum number of steps to take.
            A value of 0 means step infinitely. Internally, this number is stored in
            an unsigned 32 bit variable, so the user can specify a larger number of
            steps.
        '''
        return self.execute_ack('step %d %d %d' %(dir, speed, steps))

    def sweep(self, id, speed, steps):
        ''' The sweep command is used to sweep a bipolar motor, for step
            number of steps, at speed (0-100), thus providing a sweeping motion.
            The initial rotational direction of sweep is in the CW direction.
            Upon initial receipt of the command, the firmware will sweep the
            motor for 1/2 of the number of steps specified, starting in a CW
            direction. Once that number of steps has occurred, the sweep
            direction will change, and subsequent sweeps will rotate for the full
            amount of steps.     Thus, the starting point for the motor is in the
            middle of each sweep. You may stop the sweep by either issuing a sweep 
            command w a 0 speed, or simply sending a stop command.
        '''
        return self.execute_ack('sweep %d %d %d' %(id, speed, steps))

    def sensor(self, id):
        ''' Usage 1: reading = sensor(id)
            Usage 2: readings = sensor([id1, id2, ..., idN])
            The sensor command returns the raw A/D (8 bit) reading from the
            analog sensor ports 0-5. Multiple values can be read at a time by
            specifying multiple pins as a list. Pin 5 is 1/3 of
            the voltage of the power supply for the SerializerTM. To calculate the
            battery voltage, simply multiply the value returned by Sensor 5 by 
            15/1028.
        '''
        if type(id) == int: id=[id]
        values = self.execute_array('sensor %s' %' '.join(map(str, id)))
        n = len(values)
        for i in range(n):
            self.analog_sensor_cache[id[i]] = values[i]
        if n == 1:
            return values[0]
        else:
            return values
        
    def get_analog(self, id):
        return self.sensor(id)
    
    def get_all_analog(self):
        ''' Return the readings from all analog ports.
        '''
        return self.sensor(range(self.N_ANALOG_PORTS))

    def servo(self, id, pos):
        ''' Usage 1: servo(id, pos)
            Usage 2: servo([id1, id2, ... , idN], [pos1, pos2, ..., posN)     
            The servo command sets a servo connected to General Purpose I/O
            port the specified position. The value of the position can range from 
            -99 to 100, where 0 is the center position. Setting the position to -100
            will disable the servo, allowing it to turn freely by hand.
            Each parameter is a <servo id:position> pair, where the servo
            id can be 1,2,3,4,5, or 6.
        '''
        if type(id) == int: id = [id]
        if type(pos) == int: pos = [pos]          
        return self.execute_ack('servo %s' %' '.join(map(lambda x: '%d:%d' %x, zip(id, pos))))

    def sp03(self, msg, i2c_addr=None):
        ''' Usage1: sp03(msg)
            Usage2: sp03(msg, ic2_addr)
            The sp03 command instructs a Devantech SP03 Speech Synthesizer to
            speak the appropriate phrase. If a character representing a number in
            the range of 0 to 30, then the SP03 will speak previously programmed
            canned phrases. If a phrase is sent, then it will speak the phrase. An
            optional I2C address can also be specified. Otherwise, the default I2C
            address of 0xC4.
         '''
        return self.execute_ack('sp03 0x%X %s' %(i2c_addr, msg) if i2c_addr else 'sp03 %s' %msg)

    def srf04(self, triggerPin, outputPin):
        ''' The srf04 command queries an SRF04 sonar sensor
            connected to the General Purpose I/O lines triggerPin and outputPin,
            for a distance and returns it in the units configured (default is English
            inches). If the Serializer units are configured (using cfg units) for raw mode,
            srf04 returns readings in units of 0.4us, and the max distance returned
            is 65000 (out of range). When configured for English units, max
            distance returned is 100 inches (out of range), and when configured
            for Metric units, max distance returned is 255 (out of range). NOTE:
            Sonar distance resolution is integer based.
        '''
        return self.execute_int('srf04 %d %d' %(triggerPin, outputPin))

    def srf05(self, pinId):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns it in the units configured (default is English - inches).
            If the Serializer units are configured (using cfg units) for raw mode,
            pping and srf05 return readings in units of 0.4us, and the max
            distance returned is 65000 (out of range). When configured for
            English units, max distance returned is 100 inches (out of range), and
            when configured for Metric units, max distance returned is 255 (out of
            range). Sonar distance resolution is integer based.
        '''
        return self.execute_int('srf05 %d' %pinId);

    def pping(self, pinId):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns it in the units configured (default is English - inches).
            If the Serializer units are configured (using cfg units) for raw mode,
            pping and srf05 return readings in units of 0.4us, and the max
            distance returned is 65000 (out of range). When configured for
            English units, max distance returned is 100 inches (out of range), and
            when configured for Metric units, max distance returned is 255 (out of
            range). Sonar distance resolution is integer based.
        '''
        value = self.execute_int('pping %d' %pinId);
        self.digital_sensor_cache[pinId] = value
        return value

    def srf08(self, i2c_addr=None):
        ''' Usage1: srf08()
            Usage2: srf08(ic2_addr)
            The srf08/srf10 command queries a Devantech SRF08/SRF10 sonar
            sensor at address i2c_addr for a distance reading in the units
            configured (default is English - inches).  The i2cAddr parameter is
            optional, and defaults to 0xE0 for both sensors. The i2c address can
            be changed for any i2c module using the i2cp command.  Sonar
            distance resolution is integer based.  If the Serializer units are 
            configured (using cfg units) for raw mode, srf08 and srf10 return
            readings in units of 1us.
         '''
        return self.execute_int('srf08 0x%X' %i2c_addr if i2c_addr else 'srf08')

    def srf10(self, i2caddr=None):
        ''' Usage1: srf10()
            Usage2: srf10(ic2_addr)
            The srf08/srf10 command queries a Devantech SRF08/SRF10 sonar
            sensor at address ic2_addr for a distance reading in the units
            configured (default is English - inches). The ic2_addr parameter is
            optional, and defaults to 0xE0 for both sensors. The i2c address can
            be changed for any i2c module using the i2cp command. Sonar
            distance resolution is integer based. If the Serializer units are
            configured (using cfg units) for raw mode, srf08 and srf10 return
            readings in units of 1us.
         '''
        return self.execute_int('srf10 0x%X' %i2caddr if i2caddr else 'srf10')

    def tpa81(self, i2caddr=None):
        ''' Usage1: tpa81()
            Usage2: tpa81(ic2_addr) 
            The tpa81 command queries a Devantech TPA81 thermopile sensor for
            temperature values. It returns 8 temperature values.
        '''
        return self.execute('tpa81 0x%X' %i2caddr if i2caddr else 'tpa81')

    def vel(self):
        ''' The vel command returns the left and right wheel velocities. The
            velocity returned is based on the PIDL parameter configuration.  The
            units are encoder ticks per PID loop interval.
        '''
        return self.execute_array('vel')
    
    def vel_m_per_s(self):
        ''' Return the left and right wheel velocities in meters per second.
        '''
        [left_ticks_per_loop, right_ticks_per_loop] = self.vel()
        left_revs_per_second = float(left_ticks_per_loop) / self.encoder_resolution / self.loop_interval
        right_revs_per_second = float(right_ticks_per_loop) / self.encoder_resolution / self.loop_interval
        if self.units == 0:
            left_m_s = left_revs_per_second * self.wheel_diameter * math.pi
            right_m_s = right_revs_per_second * self.wheel_diameter * math.pi
        elif self.units == 1:
            left_m_s = left_revs_per_second * self.wheel_diameter * 2.54 * math.pi / 100
            right_m_s = right_revs_per_second * self.wheel_diameter * 2.54 * math.pi / 100
        return list([left_m_s, right_m_s])
    
    def vel_mph(self):
        ''' Return the left and right wheel velocities in miles per hour.
        '''
        [left_m_s, right_m_s] = self.vel_m_per_s()
        m_s_2_mph = 2.25
        return list([left_m_s * m_s_2_mph, right_m_s * m_s_2_mph])
    
    def vel_fps(self):
        ''' Return the left and right wheel velocities in feet per second.
        '''
        [left_m_s, right_m_s] = self.vel_m_per_s()
        m_s_2_fps = 3.2808
        return list([left_m_s * m_s_2_fps, right_m_s * m_s_2_fps]) 

    def restore(self):
        ''' Restores the factory default settings, and resets the board. NOTE:
            This will erase any configurations you have saved to EEPROM,
            including VPID, DPID, and baud rate settings.
        '''
        print self.execute('restore')

    def line(self, addr, newaddr=None, seven=False):
        ''' Queries a RoboticsConnection Line Following Sensor at address addr.
            If the -a option is specified, then the address of the module will be
            changed to the new address associated w the -a switch.
            If the optional 7 is appended to the end of the line command, e.g.
            line7, then two additional values will be returned from those Line
            Following Sensors (manufactured after 11/1/07) which have additional
            sensor inputs on the sides of the board. This can be used to read
            additional Single Line Following sensors, or read any type of on/off
            momentary switch, such those used for bumpers.
        '''
        cmd = ('line7 %d' if seven else 'line %d') %addr
        if newaddr: cmd += ' -a %d' %newaddr
        return self.execute_array(cmd)

    def i2c(self, op, addr, data=None):
        ''' Usage1: i2c(op, addr)
            Usage2: i2c(op, addr, data)
            The flexible i2c command allows you to execute a generic i2c read, or
            write command to the specified device at address addr. Depending on
            whether you issue a read or write, additional parameters vary.
        '''
        cmd = 'i2c %c 0x%X ' %(op, addr)
        if data:
            cmd += ''.join(data)
        if op == 'w':
            return self.execute_ack(cmd)
        else:
            return self.execute_array(cmd)
        
    def voltage(self, cached=False):
        if cached and self.analog_sensor_cache[5] != None:
            return self.analog_sensor_cache[5] * 15. / 1024.
        else:
            try:
                return self.sensor(5) * 15. / 1024.
            except:
                pass
        
    def set_wheel_diameter(self, diameter):
        self.wheel_diameter = diameter
        
    def get_wheel_diameter(self):
        return self.wheel_diameter
    
    def set_wheel_track(self, track):
        self.wheel_track = track
        
    def get_wheel_track(self):
        return self.wheel_track
    
    def set_gear_reduction(self, ratio):
        self.gear_reduction = ratio
        
    def get_gear_reduction(self):
        return self.gear_reduction
    
    def set_encoder_resolution(self, ticks):
        self.encoder_resolution = ticks
        
    def get_encoder_resolution(self):
        return self.encoder_resolution
    
    def get_Ping(self, pin, cached=False):
        ''' Get the distance reading from a Ping sonarl sensor on the given GPIO pin.
        '''
        if cached and self.digital_sensor_cache[pin] != None:
            value = self.digital_sensor_cache[pin]
        else:
            value = self.pping(pin)
        return value
    
    def get_GP2D12(self, pin, cached=False):
        ''' Get the distance reading from a GP2D12 IR sensor on the given analog pin.
        '''
        if cached and self.analog_sensor_cache[pin] != None:
            value = self.analog_sensor_cache[pin]
        else:
            value = self.sensor(pin)
        try:
            distance = (6787 / (value - 3)) - 4
        except:
            distance = 80
        if distance > 80: distance = 80
        if distance < 10: distance = 10   
        if self.units == 0:
            return distance
        elif self.units == 1:
            return distance / 2.54
        else:
            return value
        
    def get_PhidgetsTemperature(self, pin, cached=False, units="F"):
        ''' Get the temperature from a Phidgets Temperature sensor on an analog sensor port and return
            the reading in either Farhenheit or Celcius depending on the units argument.
        '''
        self.temp_units = units
        if cached and self.analog_sensor_cache[pin] != None:
            value = self.analog_sensor_cache[pin]
        else:
            value = self.sensor(pin)
        tempC = (value - 200.) / 4.
        if self.temp_units == "C":
            return tempC
        else:
            return 9. * tempC / 5. + 32.
        
    def get_PhidgetsVoltage(self, pin, cached=False):
        ''' Get the voltage from a Phidgets Voltage sensor on an analog sensor port.
        '''
        
        if cached and self.analog_sensor_cache[pin] != None:
            value = self.analog_sensor_cache[pin]
        else:
            value = self.sensor(pin)
        return 0.06 * (value - 500.)
    
    def get_PhidgetsCurrent(self, pin, cached=False, model=20, ac_dc="dc"):
        if cached and self.analog_sensor_cache[pin] != None:
            value = self.analog_sensor_cache[pin]
        else:
            value = self.sensor(pin)
        if model == 20:
            if ac_dc == "dc":
                return 0.05 * (value - 500.)
            else:
                return 0.025 * value
        else:
            if ac_dc == "dc":
                return 0.125 * (value - 500.)
            else:
                return 0.625 * value        
         
    def travel_distance(self, dist, vel):
        ''' Move forward or backward 'dist' (inches or meters depending on units) at speed 'vel'.  Use negative distances
            to move backward.
        '''

        revs_per_second = float(vel) / (self.wheel_diameter * math.pi)
            
        ticks_per_loop = revs_per_second * self.encoder_resolution * self.loop_interval
        vel = (int(ticks_per_loop))
            
        revs = dist / (self.wheel_diameter * math.pi)
        ticks = revs * self.encoder_resolution * self.gear_reduction
        self.digo([1, 2], [ticks, ticks], [vel, vel])
        
    def mogo(self, id, vel):
        ''' Usage 1: mogo(id, vel)
            Usage 2: mogo([id1, id2], [vel1, vel2])
            The mogo command sets motor speed using one or more complex
            parameters containing a <motorId:spd> value pair.
            The motorId can be either 1 or 2, which corresponds to the Motor
            Terminal port.
            The vel value specifies the motor velocity, and it's range depends on
            your VPID settings. See the VPID parameters section below to
            determine your MAX velocity. A positive value rotates the motors in
            one direction, which a negative value rotates the motors in the
            opposite direction.
            You will have to determine which direction is positive for your motors,
            and connect the motors wires to the terminals on the Serializer board
            in the appropriate configuration.
        '''
        if type(id) == int: id = [id]
        if type(vel) == int: vel = [vel]          
        return self.execute_ack('mogo %s' %' '.join(map(lambda x: '%d:%d' %x, zip(id, vel))))
    
    def mogo_m_per_s(self, id, vel):
        ''' Set the motor speeds in meters per second.
        '''
        if type(id) != list: id = [id]
        if type(vel) != list: vel = [vel]
        spd = list()
        for v in vel:
            if self.units == 0:
                revs_per_second = float(v) / (self.wheel_diameter * math.pi)
            elif self.units == 1:
                revs_per_second = float(v) / (self.wheel_diameter * math.pi * 2.54 / 100)
            ticks_per_loop = revs_per_second * self.encoder_resolution * self.loop_interval * self.gear_reduction
            spd.append(int(ticks_per_loop))
                                                    
        return self.execute_ack('mogo %s' %' '.join(map(lambda x: '%d:%d' %x, zip(id, spd))))
        
    def travel_at_speed(self, id, vel):
        ''' Move forward or backward at speed 'vel' in meters per second.  Use negative speeds
            to move backward.  This is just an alias for mogo_m_per_s so see that function for
            more details.
        '''
        self.mogo_m_per_s(id, vel)
        
    def rotate(self, angle, vel):
        ''' Rotate the robot through 'angle' degrees or radians at speed 'vel'.  Use negative angles to rotate
            in the other direction.
        '''
        revs_per_second = float(vel) / (self.wheel_diameter * math.pi)
        
        if self.units == 0:
            rotation_fraction = angle / (2.0 * math.pi)
        elif self.units == 1:
            rotation_fraction = angle / 360.
            
        ticks_per_loop = revs_per_second * self.encoder_resolution * self.loop_interval * self.gear_reduction
        vel = (int(ticks_per_loop))
        
        full_rotation_dist = self.wheel_track * math.pi
        rotation_dist = rotation_fraction * full_rotation_dist
        revs = rotation_dist / (self.wheel_diameter * math.pi)
        ticks = revs * self.encoder_resolution  * self.gear_reduction
        self.digo([1, 2], [ticks, -ticks], [vel, vel])
        
    def rotate_at_speed(self, vel):
        ''' Rotate the robot continously at speed 'vel' radians or degrees per second.  Use negative speed to rotate
            in the other direction.
        '''
            
        if self.units == 1:
            vel = vel / 180. * math.pi
            
        # Check that the user does not mistaken degrees for radians.
        if vel > 1.0:
            print "That is a rather high rotation rate. Are you sure you specified rotation velocity in the correct units?"
            print "Degrees per second for English units and radians per second for metric."
            print "Keep in mind that 1 radian per second is about 60 degrees per second."
            os._exit(1)
            
        if self.units == 1:
            full_rotation_dist = self.wheel_track * 2.54 / 100 * math.pi
        else:
            full_rotation_dist = self.wheel_track * math.pi
            
        revs_per_second = float(vel) / (2.0 * math.pi)
        vel = revs_per_second * full_rotation_dist
        
        self.mogo_m_per_s([1, 2], [vel, -vel])
        
    def twist(self, twist):
        angle = math.atan2(twist.linear.y, twist.linear.x)
        self.rotate(angle, twist.angular.z)
        while self.get_pids():
            time.sleep(0.05)
        speed = math.sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y)
        self.mogo_m_per_s([1, 2], [speed, speed])
    
class Linear:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        
class Angular:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        
class Twist:
    def __init__(self):
        self.linear = Linear()
        self.angular = Angular()      

class PhidgetsTemperature():
    def __init__(self, serializer, pin, units="F"):
        ''' Usage: myTemp = PhidgetsTemperature(serializer, pin)
                   reading = myTemp.value()
                   reading = myTemp.value(cached=True) # gets value from the cache
            The Phidgets Temperature Sensor class wraps an analog sensor port and converts the raw
            sensor reading to either Farhenheit or Celcius depending on the units argument.
        '''
        self.serializer = serializer
        self.pin = pin
        self.temp_units = units
    
    def value(self, cached=False, units="F"):   
        return self.serializer.get_PhidgetsTemperature(self.pin, cached, units)
    

class PhidgetsVoltage():
    def __init__(self, serializer, pin):
        ''' Usage: myVolts = PhidgetsVoltage(serializer, pin)
                   reading = myVolts.value()
                   reading = myVolts.value(cached=True) # gets value from the cache
            The Phidgets Voltage Sensor class wraps an analog sensor port and converts the raw
            sensor reading to volts.
        '''
        self.serializer = serializer
        self.pin = pin
    
    def value(self, cached=False):
        return self.serializer.get_PhidgetsVoltage(self.pin, cached)
 

class PhidgetsCurrent():
    def __init__(self, serializer, pin, model=20, ac_dc="dc"):
        ''' Usage: myAmps = PhidgetsCurrent(serializer, pin)
                   reading = myAmps.value()
                   reading = myAmps.value(cached=True) # gets value from the cache
            The Phidgets Current Sensor class wraps an analog sensor port and converts the raw
            sensor reading to amps.  Note there are two models of the Phidgets current sensor, 
            one with a 20 amp max and the other with a 50 amp max. Also, either model can 
            measure either DC or AC current.
        '''
        self.serializer = serializer
        self.pin = pin
        self.model = model
        self.ac_dc = ac_dc
        
    def value(self, cached=False):
        return self.serializer.get_PhidgetsCurrent(self.pin, cached, self.model, self.ac_dc)        


class Ping():
    def __init__(self, serializer, pin):
        ''' Usage: myPing = Ping(serializer, pin)
                   reading = myPing.value()
                   reading = myPing.value(cached=True) # gets value from the cache
            The Parallax Ping Sonar Sensor class wraps a digital sensor port returns the value
            from the pping() command which in turn is in inches or cm depending on the units settings.
        '''
        self.serializer = serializer
        self.pin = pin
    
    def value(self, cached=False):   
        return self.serializer.get_Ping(self.pin, cached)
    
class GP2D12():
    def __init__(self, serializer, pin):
        ''' Usage: myIR = GP2D12(serializer, pin)
                   reading = myIR.value()
                   reading = myIR.value(cached=True) # gets value from the cache
            The Sharp GPD12 IR Sensor class wraps an analog sensor port and converts the raw
            sensor reading to either inches or cm depending on the units settings.
        '''
        self.serializer = serializer
        self.pin = pin

    def value(self, cached=False):   
        return self.serializer.get_GP2D12(self.pin, cached)
 

""" Basic test for connectivity """
if __name__ == "__main__":
    import time
    if os.name == "posix":
        #portName = "/dev/ttyUSB0"
        portName = "/dev/rfcomm0" # For bluetooth on Linux
        # Note: On Linux, after connecting to the Bluetooth adapter, run the command
        # sudo rfcomm bind /dev/rfcomm0
    else:
        portName = "COM12" # Windows style COM port.
        
    baudRate = 19200
  
    mySerializer = Serializer(port=portName, baudrate=baudRate, timeout=5)
    mySerializer.connect()
    
    print "Firmware Version", mySerializer.fw()
    print "Units", mySerializer.get_units()
    print "Baudrate", mySerializer.get_baud()
    print "VPID", mySerializer.get_vpid()
    print "DPID", mySerializer.get_dpid()
    print "Voltage", mySerializer.voltage()
    
    print "Connection test successful, now shutting down...",
    
    mySerializer.stop()
    mySerializer.close()
    
    print "Done."
    