#!/usr/bin/env python

#  Software License Agreement (Apache License)
#  
#  Copyright (c) 2014, Southwest Research Institute
#  
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#  
#  http://www.apache.org/licenses/LICENSE-2.0
#  
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from robo_cylinder.srv import *

import rospy
import time
import sys
import serial

from std_msgs.msg import Float32

# configure the serial connections
def ser_init():
    global ser
    ser = serial.Serial(
        port=rospy.get_param('~port'),
    	baudrate=9600,
    	parity=serial.PARITY_NONE,
    	stopbits=serial.STOPBITS_ONE,
    	bytesize=serial.EIGHTBITS,
    	timeout=3
    )
    return

# define global character shortcuts for checksums
def char_init():
    global stx
    global etx
    global zero

    zero = '0'
    # start and end characters for each checksum
    stx = "02".decode("hex")
    etx = "03".decode("hex")

# define input parameters specific to this robo cylinder
# axis can be changed to 0-15. Lead screw is somewhere between 2-10mm
def param_init():
    global axis
    global axis_str
    global axis_val
    global lead

    axis = rospy.get_param('~axis')
    axis_str = hex(axis).upper()[2:]
    axis_val = int(axis_str.encode("hex"), 16)
    lead = rospy.get_param('~lead')

# BCC lsb calculator to check communication integrity
def bcc_calc(bcc_int):
    bcc = (~bcc_int & 0xFFF) + 1        # 2's complement calculation
    for i in range(11, 7, -1):
        if bcc > 2**i:
            bcc -= 2**i                 # takes the LSB of the integer
    bcc = hex(bcc).upper()[2:]          # converts the integer to hex characters
    if len(bcc) == 1: bcc = '0' + bcc   # protocol needs BCC to be two characters
    return bcc

# Retrieve status signal
def status(req):
    msg = axis_str + 'n' + 10*zero # creates message
    # BCC is calculated based on the ascii values of the 12-character message (everything but the stx, bcc, and etx)
    bcc_int = axis_val + ord('n') + 10*ord(zero) # formula for bcc char
    bcc = bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init
    csum = stx + msg + bcc + etx # creates the package to send into the controller
    ser.write(csum) # writes to the serial communication
    rospy.loginfo('Status request checksum sent: %s\n'%csum) # allows user to know what message package was sent
    return True

# Power signal
def power(req):
    io = str(req.io) # requests io information
    msg = axis_str + 'q' + io + 9*zero # creates message
    bcc_int = axis_val + ord('q') + ord(io) + 9*ord(zero) # formula for bcc char
    bcc = bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init
    csum = stx + msg + bcc + etx # creates the package to send into the controller
    ser.write(csum) # writes to the serial communication
    rospy.loginfo("Power checksum sent: %s\n"%csum) # allows user to know what message package was sent
    return True

# Homing signal
def home(req):
    init_pos = pos_meters # initial position of car is defined here.
    if init_pos <= 0.005: #  if the position is already home, it is true already. no need to proceed
        print'Currently at home!'
        return True
    while (pos_meters == init_pos): # while false, keeping sending command until true
        direction = '9' # sets direction
        msg = axis_str + 'o' + zero + direction + 8*zero # creates message
        bcc_int = axis_val + ord('o') + ord(direction) + 9*ord(zero) # formula for bcc char
        bcc = bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init
        csum = stx + msg + bcc + etx # creates the package to send into the controller
        ser.flushInput() #  clear buffer in case the serial responses get jumbled
        ser.write(csum) # writes to the serial communication
        rospy.loginfo('Homing command checksum sent: %s\n'%csum) # allows user to know what message package was sent
        rospy.sleep(4) # gives the car time to move (4 seconds)
    if (pos_meters != init_pos):
        while (pos_meters > 0.005):
            rospy.sleep(0.10)
        if (pos_meters <= 0.005): # if it is home, return true.
            print'Home reached!'
            return True



# Absolute positioning signal
def abs_move(pos, converted):
    meters = pos*lead/8000.0/1000.0 # the 8000 is a constant from protocol conversion. 1000 is conv from mm to meters
    init_pos = pos_meters
    init_pulse = init_pos*1000.0*8000.0/lead # converts initial position to pulses
    pos_pulse = pos_meters*1000.0*8000.0/lead # converts position to pulses

    # # #  MOVE_METERS # # #
    if converted: # if move_meters is called, must convert pulses to position
        max_meter= meters+0.0000075 # max value from the meter, sometimes is a tiny bit off
        min_meter = meters-0.0000015 # min value from the meter, sometimes is a tiny bit off
        if (init_pos <= max_meter and init_pos >= min_meter): # if we're already there, it's true
            rospy.loginfo('Position in meters: %s\n'%pos_meters) # displays position in meters
            return True
        while (pos_meters == init_pos): # while false, keeping sending command until true
            position = pos # var position is the passed in var pos from command line
            #protocol subtracts position from FFFFFFFF if the robot homes to the motor end
            position = hex(16**8 - 1 - int(round(position))).upper()[2:10]
            msg = axis_str + 'a' + position + 2*zero # creates message
            pos_int = 0 # clears string
            for i in range(0, 8):               # adds up the ascii values of each character of the hex position
                pos_int += int(position[i].encode("hex"), 16)
            bcc_int = axis_val + ord('a') + pos_int + 2*ord(zero) # formula for bcc char
            bcc = bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init
            csum = stx + msg + bcc + etx # creates the package to send into the controller
            ser.flushInput() #  clear buffer in case the serial responses get jumbled
            ser.write(csum) # writes to the serial communication
            rospy.loginfo('Position command checksum sent: %s\n'%csum) # allows user to know what message package was sent
            rospy.sleep(4) # gives the car time to move (4 seconds)
        if (pos_meters != init_pos):
            while (pos_meters > max_meter or pos_meters < min_meter):
                rospy.sleep(0.10)
            if (pos_meters <= max_meter and pos_meters >= min_meter): # if the position of the car is basically at the desired position, it's true.
                rospy.loginfo('Position in meters: %s\n'%pos_meters) # displays position in meters
                return True

    # # #  MOVE_PULSES # # #  -> slight problem when asked to go to 480,000 pulses but loop takes care of it.
    else:
        min_pulse = pos-1000.0 # min pulse value for basically same position
        max_pulse = pos+1000.0 # max pulse value for basically same position
        if (init_pulse <= max_pulse and init_pulse >= min_pulse): # if we're already there, it's true
            rospy.loginfo('Position in pulses: %s\n'%pos_pulse) # displays position in pulses
            return True
        while (pos_pulse == init_pulse): # while false, keeping sending command until true
            position = pos # var position is the passed in var pos from command line
            #protocol subtracts position from FFFFFFFF if the robot homes to the motor end
            position = hex(16**8 - 1 - int(round(position))).upper()[2:10]
            msg = axis_str + 'a' + position + 2*zero # creates message
            pos_int = 0 # clears string
            for i in range(0, 8):               # adds up the ascii values of each character of the hex position
                pos_int += int(position[i].encode("hex"), 16)
            bcc_int = axis_val + ord('a') + pos_int + 2*ord(zero) # formula for bcc char
            bcc = bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init
            csum = stx + msg + bcc + etx # creates the package to send into the controller
            ser.flushInput() #  clear buffer in case the serial responses get jumbled
            ser.write(csum) # writes to the serial communication
            rospy.loginfo('Position command checksum sent: %s\n'%csum) # allows user to know what message package was sent
            rospy.sleep(4) # gives the car time to move (4 seconds)
        if (pos_pulse != init_pulse):
            while (pos_pulse > max_pulse or pos_pulse < min_pulse):
                rospy.sleep(0.10)
            if (pos_pulse <= max_pulse and pos_pulse >= min_pulse): # if the position of the car is basically at the desired position, it's true.
                rospy.loginfo('Position in pulses: %s\n'%pos_pulse) # displays position in pulses
                return True



# Velocity and acceleration change signal
def vel_acc(req):
    vel = req.vel # gets velocity from command line
    acc = req.acc # gets acceleration from command line

    # # DEBUG
    rospy.loginfo('Current velocity: %s\n'%vel)

    # conversions defined in protocol instructions
    vel = vel*100*300/lead
    vel = hex(int(vel)).upper()[2:]

    acc = acc/9.8*5883.99/lead
    acc = hex(int(acc)).upper()[2:]

    # protocol requires length of 4 characters
    while len(acc) < 4:
        acc = '0' + acc
    while len(vel) < 4:
        vel = '0' + vel

    msg = axis_str + 'v' + '2' + vel + acc + zero # creates message
    vel_acc_int = 0 # clears vel_acc_init string
    for i in range (0, 4): # fills the vel_acc_init string
        vel_acc_int += int(vel[i].encode("hex"), 16)
        vel_acc_int += int(acc[i].encode("hex"), 16)
    bcc_int = axis_val + ord('v') + ord('2') + vel_acc_int + ord(zero) # formula for bcc char
    bcc = bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init
    csum = stx + msg + bcc + etx # creates the package to send into the controller
    ser.flushInput()		 #  clear buffer in case the serial responses get jumbled
    ser.write(csum) # writes to the serial communication
    rospy.loginfo('Velocity and acceleration change checksum sent: %s\n'%csum) # allows user to know what message package was sent
    return True

# Position inquiry
def pos_inq(pos_meters):
    msg = axis_str + 'R4' + 4*zero + '74' + 3*zero # creates message
    bcc_int = axis_val + ord('R') + 2*ord('4') + ord('7') + 7*ord(zero) # formula for bcc char
    bcc = bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init
    csum = stx + msg + bcc + etx # creates package to send into the controller
    ser.flushInput()		 #  clear buffer in case the serial responses get jumbled
    ser.write(csum) # writes to the serial communication
    response = ser.read(16) # reads response from controller
    if len(response) == 0: response = '_' # if no response, the response is blank
    if response[0] == stx: # if the response is actually a message package
       try:
          pos_pulses = int(response[5:13], 16) # reads the response and collects pulse information
          pos_temp = float(((16**8-1)-pos_pulses)*10/8000.0/1000.0) # calculates the meters from the pulses
	  if pos_temp < 10: # if the meters position data is under 10 (aka correct)
	     pos_meters = pos_temp       #  a check to ensure position isn't read incorrectly. Occassionally happens at the beginning of a move
       except ValueError:
	  print 'ValueError' # if the pos_temp gives an incorrect or impossible value, print Value Error so the user knows
    else:
	ser.flushInput()		 #  clear buffer in case the serial responses get jumbled
    return pos_meters # returns the global var pos_meters

# these handlers vary depending on if the input is pulses or meters
def handle_move_pulses(req):
    return MovePulsesResponse(abs_move(req.pulses, 0)) # calls to abs_pos

def handle_move_meters(req):
    pulses = int(req.meters*1000*8000/lead) # converts desired meters position to pulses for abs_pos function
    return MoveMetersResponse(abs_move(pulses, 1)) # calls to abs_pos

# current position publisher
def talker():
    global pos_meters
    pos_meters = 1
    pub = rospy.Publisher('/car/pos', Float32, queue_size=10)
    rate = rospy.Rate(10)                #  10hz
    while not rospy.is_shutdown():
        pos_meters = pos_inq(pos_meters) # calls pos_ing for pos_meters while ros is running
	pub.publish(pos_meters) #  publishes pos_meters
        rate.sleep()

#  all functions and services initializations
def services_init():
    rospy.init_node('robo_cylinder_services')
    ser_init()
    char_init()
    param_init()
    status_s = rospy.Service('status_service', StatusUpdate, status)
    power_io_s = rospy.Service('power_io', PowerIO, power)
    home_s = rospy.Service('home_service', HomeCmd, home)
    move_pulses_s = rospy.Service('move_pulses', MovePulses, handle_move_pulses)
    move_meters_s = rospy.Service('move_meters', MoveMeters, handle_move_meters)
    vel_acc_s = rospy.Service('vel_acc', VelAcc, vel_acc)    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

    rospy.spin() #  wait for everything to be done 

if __name__ == "__main__":
    services_init() #  calls the service_init function if the name is main
