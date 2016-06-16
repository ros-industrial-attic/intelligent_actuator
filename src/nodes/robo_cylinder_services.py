#!/usr/bin/env python

from robo_cylinder.srv import *

import rospy
import time
import sys
import serial

#configure the serial connections

ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=3
)

##ser.close()
##ser.open()
ser.isOpen()

#start and end characters for each checksum
stx = "02".decode("hex")
etx = "03".decode("hex")

#input parameters specific to this robo cylinder
#axis can be changed to 0-15 but ours operates on axis 0. Lead screw is supposedly $
axis = 0
axis_str = hex(axis).upper()[2:]
axis_val = int(axis_str.encode("hex"), 16)
lead = 10

zero = '0'

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
def status():
    msg = axis_str + 'n' + 10*zero
    #BCC is calculated based on the ascii values of the 12-character message (everything but the stx, bcc, and etx)
    bcc_int = axis_val + ord('n') + 10*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    print 'Status checksum sent: %s\n'%csum
    return csum

#Power signal
def power(io):
    io = str(io)
    msg = axis_str + 'q' + io + 9*zero
    bcc_int = axis_val + ord('q') + ord(io) + 9*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    if io == '1':
        print 'Power on checksum sent: %s\n'%csum
    elif io == '0':
        print 'Power off checksum sent: %s\n'%csum
    return csum

#Homing signal
def home():
    direction = '9'
    msg = axis_str + 'o' + zero + direction + 8*zero
    bcc_int = axis_val + ord('o') + ord(direction) + 9*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    print 'Homing checksum sent: %s\n'%csum
    time.sleep(5)                       #homing usually takes a few extra seconds to complete
    return csum

#Absolute positioning signal
def abs_move(position):
    a = 'a'
    #protocol subtracts position from FFFFFFFF if the robot homes to the motor end
    position = hex(16**8 - 1 - int(round(position))).upper()[2:10]
    msg = axis_str + a + position + 2*zero
    pos_int = 0
    for i in range(0, 8):               #adds up each ascii values of each character of the hex position
        pos_int += int(position[i].encode("hex"), 16)
    bcc_int = axis_val + ord(a) + pos_int + 2*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    ser.write(csum)
    print 'Position checksum sent: %s\n'%csum
    time.sleep(1)
    return csum

#Velocity and acceleration change signal
def vel_acc(vel, acc):
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
    print 'Velocity and acceleration checksum sent: %s\n'%csum
    return csum

def handle_status(req):
    print "Returning checksum for status update request."
    return StatusUpdateResponse(status())

def handle_power_io(req):
    if req.io == 0:
        print "Returning checksum for power off"
    elif req.io == 1:
        print "Returning checksum for power on"
    return PowerIOResponse(power(req.io))

def handle_home(req):
    print "Returning checksum for home command"
    return HomeCmdResponse(home())

def handle_move_pulses(req):
    print "Returning checksum for move of %d pulses"%req.pulses
    return MovePulsesResponse(abs_move(req.pulses))

def handle_move_meters(req):
    print "Returning checksum for move of %d meters"%req.meters
    pulses = int(req.meters*1000*8000/lead)
    return MoveMetersResponse(abs_move(pulses))

def handle_vel_acc(req):
    print "Returning checksum for velocity change to %d and acceleration to %d"%(req.vel, req.acc)
    return VelAccResponse(vel_acc(req.vel, req.acc))

def services_init():
    rospy.init_node('robo_cylinder_services')
    status_s = rospy.Service('status_service', StatusUpdate, handle_status)
    power_io_s = rospy.Service('power_io', PowerIO, handle_power_io)
    home_s = rospy.Service('home_service', HomeCmd, handle_home)
    move_pulses_s = rospy.Service('move_pulses', MovePulses, handle_move_pulses)
    move_meters_s = rospy.Service('move_meters', MoveMeters, handle_move_meters)
    vel_acc_s = rospy.Service('vel_acc', VelAcc, handle_vel_acc)    
    print "All services ready."
    rospy.spin()

if __name__ == "__main__":
    services_init()
