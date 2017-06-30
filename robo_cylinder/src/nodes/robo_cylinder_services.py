#!/usr/bin/env python

# Software License Agreement (Apache License)
# 
# Copyright (c) 2014, Southwest Research Institute
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
# http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from robo_cylinder.srv import *

import rospy
import time
import sys
import serial

from std_msgs.msg import Float32

#configure the serial connections
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

#define global character shortcuts for checksums
def char_init():
    global stx
    global etx
    global zero

    zero = '0'
    #start and end characters for each checksum
    stx = "02".decode("hex")
    etx = "03".decode("hex")

#define input parameters specific to this robo cylinder
#axis can be changed to 0-15. Lead screw is somewhere between 2-10mm
def param_init():
    global axis
    global axis_str
    global axis_val
    global lead

    axis = rospy.get_param('~axis')
    axis_str = hex(axis).upper()[2:]
    axis_val = int(axis_str.encode("hex"), 16)
    lead = rospy.get_param('~lead')

#BCC lsb calculator to check communication integrity
def bcc_calc(bcc_int):
    bcc = (~bcc_int & 0xFFF) + 1        #2's complement calculation
    for i in range(11, 7, -1):
        if bcc > 2**i:
            bcc -= 2**i                 #takes the LSB of the integer
    bcc = hex(bcc).upper()[2:]          #converts the integer to hex characters
    if len(bcc) == 1: bcc = '0' + bcc   #protocol needs BCC to be two characters
    return bcc

#Retrieve status signal
def status(req):
    msg = axis_str + 'n' + 10*zero
    #BCC is calculated based on the ascii values of the 12-character message (everything but the stx, bcc, and etx)
    bcc_int = axis_val + ord('n') + 10*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    rospy.loginfo('Status request checksum sent: %s\n'%csum)
    return True

#Power signal
def power(req):
    io = str(req.io)
    msg = axis_str + 'q' + io + 9*zero
    bcc_int = axis_val + ord('q') + ord(io) + 9*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    rospy.loginfo("Power checksum sent: %s\n"%csum)
    return True

#Homing signal
def home(req):
    direction = '9'
    msg = axis_str + 'o' + zero + direction + 8*zero
    bcc_int = axis_val + ord('o') + ord(direction) + 9*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    rospy.loginfo('Homing command checksum sent: %s\n'%csum)
    return True

#Absolute positioning signal
def abs_move(pos, converted):
    position = pos
    #protocol subtracts position from FFFFFFFF if the robot homes to the motor end
    position = hex(16**8 - 1 - int(round(position))).upper()[2:10]
    msg = axis_str + 'a' + position + 2*zero
    pos_int = 0
    for i in range(0, 8):               #adds up the ascii values of each character of the hex position
        pos_int += int(position[i].encode("hex"), 16)
    bcc_int = axis_val + ord('a') + pos_int + 2*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    rospy.loginfo('Position command checksum sent: %s\n'%csum)
    if converted:
    	meters = pos*lead/8000.0/1000.0 #the 8000 is a constant from protocol conversion. 1000 is conv from mm to meters
    return True

#Velocity and acceleration change signal
def vel_acc(req):
    vel = req.vel
    acc = req.acc

    #conversions defined in protocol instructions
    vel = vel*100*300/lead
    vel = hex(int(vel)).upper()[2:]

    acc = acc/9.8*5883.99/lead
    acc = hex(int(acc)).upper()[2:]

    #protocol requires length of 4 characters
    while len(acc) < 4:
        acc = '0' + acc
    while len(vel) < 4:
        vel = '0' + vel

    msg = axis_str + 'v' + '2' + vel + acc + zero
    vel_acc_int = 0
    for i in range (0, 4):
        vel_acc_int += int(vel[i].encode("hex"), 16)
        vel_acc_int += int(acc[i].encode("hex"), 16)
    bcc_int = axis_val + ord('v') + ord('2') + vel_acc_int + ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    rospy.loginfo('Velocity and acceleration change checksum sent: %s\n'%csum)
    return True

#Position inquiry
def pos_inq(pos_meters):
    msg = axis_str + 'R4' + 4*zero + '74' + 3*zero
    bcc_int = axis_val + ord('R') + 2*ord('4') + ord('7') + 7*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    response = ser.read(16)
    if len(response) == 0: response = '_'
    if response[0] == stx:
       try:
          pos_pulses = int(response[5:13], 16)
          pos_temp = float(((16**8-1)-pos_pulses)*10/8000.0/1000.0)
	  if pos_temp < 10:
	     pos_meters = pos_temp       # a check to ensure position isn't read incorrectly. Occassionally happens at the beginning of a move
       except ValueError:
	  print 'ValueError'
    else:
	ser.flushInput()		 # clear buffer in case the serial responses get jumbled
    return pos_meters

#these handlers vary depending on if the input is pulses or meters
def handle_move_pulses(req):
    return MovePulsesResponse(abs_move(req.pulses, 0))

def handle_move_meters(req):
    pulses = int(req.meters*1000*8000/lead)
    return MoveMetersResponse(abs_move(pulses, 1))

#current position publisher
def talker():
    global pos_meters
    pos_meters = 1
    pub = rospy.Publisher('/car/pos', Float32, queue_size=10)
    rate = rospy.Rate(10)                # 10hz
    while not rospy.is_shutdown():
        pos_meters = pos_inq(pos_meters)
	pub.publish(pos_meters)
        rate.sleep()

#all functions and services initializations
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

    rospy.spin()

if __name__ == "__main__":
    services_init()
