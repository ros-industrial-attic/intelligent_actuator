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
import threading

from std_msgs.msg import Float32

class RoboCylinder:
    def __init__(self):

        self.serial_lock = threading.Lock()
        self.parameter_lock = threading.Lock()

        port=rospy.get_param('~port')
        self.ser = serial.Serial(port, baudrate=9600, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=3)

        self.zero = '0'
        # start and end characters for each checksum
        self.stx = "02".decode("hex")
        self.etx = "03".decode("hex")

        self.axis = rospy.get_param('~axis')
        self.axis_str = hex(self.axis).upper()[2:]
        self.axis_val = int(self.axis_str.encode("hex"), 16)
        self.lead = rospy.get_param('~lead')

        self.position_meters = 0.0

        self.position_pub = rospy.Publisher('/car/pos', Float32, queue_size=10)
        self.status_s = rospy.Service('status_service', StatusUpdate, self.status)
        self.power_io_s = rospy.Service('power_io', PowerIO, self.power)
        self.home_s = rospy.Service('home_service', HomeCmd, self.home)
        self.move_pulses_s = rospy.Service('move_pulses', MovePulses, self.handle_move_pulses)
        self.move_meters_s = rospy.Service('move_meters', MoveMeters, self.handle_move_meters)
        self.vel_acc_s = rospy.Service('vel_acc', VelAcc, self.vel_acc)

        worker_thread = threading.Thread(target=self.talker())
        worker_thread.start()

    # BCC lsb calculator to check communication integrity
    def bcc_calc(self, bcc_int):
        bcc = (~bcc_int & 0xFFF) + 1        # 2's complement calculation
        for i in range(11, 7, -1):
            if bcc > 2**i:
                bcc -= 2**i                 # takes the LSB of the integer
        bcc = hex(bcc).upper()[2:]          # converts the integer to hex characters
        if len(bcc) == 1: bcc = '0' + bcc   # protocol needs BCC to be two characters
        return bcc

    def serial_read_write(self, msg):

        bcc_int = 0

        for c in range(len(msg)-1,-1,-1):
            bcc_int += ord(msg[c])
        bcc = self.bcc_calc(bcc_int) # calculates bcc based off msg and bcc_init

        csum = self.stx + msg + bcc + self.etx # creates the package to send into the controller
        lock_timeout = 1.0
        response = ""
        if self.serial_lock.acquire():
            self.ser.flushInput()
            self.ser.write(csum) # writes to the serial communication
            response = self.ser.read(16)
            self.serial_lock.release()
        else:
            rospy.logerr("Failed to acquire serial lock in %d seconds"%lock_timeout)

        return response

    # Retrieve status signal
    def status(self, req):
        msg = self.axis_str + 'n' + 10*self.zero # creates message
        self.serial_read_write(msg)
        return True

    # Power signal
    def power(self, req):
        io = str(req.io) # requests io information
        msg = self.axis_str + 'q' + io + 9*self.zero # creates message
        self.serial_read_write(msg)
        return True

    # Velocity and acceleration change signal
    def vel_acc(self, req):
        vel = req.vel # gets velocity from command line
        acc = req.acc # gets acceleration from command line

        # # DEBUG
        rospy.loginfo('Current velocity: %s\n'%vel)

        # conversions defined in protocol instructions
        vel = vel*100*300/self.lead
        vel = hex(int(vel)).upper()[2:]

        acc = acc/9.8*5883.99/self.lead
        acc = hex(int(acc)).upper()[2:]

        # protocol requires length of 4 characters
        while len(acc) < 4:
            acc = '0' + acc
        while len(vel) < 4:
            vel = '0' + vel

        msg = self.axis_str + 'v' + '2' + vel + acc + self.zero # creates message
        self.serial_read_write(msg)
        return True

    # Homing signal
    def home(self, req):
        direction = '9' # sets direction
        msg = self.axis_str + 'o' + self.zero + direction + 8*self.zero # creates message
        self.serial_read_write(msg)
        return True

    # Position inquiry
    def pos_inq(self):
        msg = self.axis_str + 'R4' + 4*self.zero + '74' + 3*self.zero # creates message
        response = self.serial_read_write(msg)

        if len(response) == 0: response = '_' # if no response, the response is blank
        if response[0] == self.stx: # if the response is actually a message package
           try:
              pos_pulses = int(response[5:13], 16) # reads the response and collects pulse information
              pos_temp = float(((16**8-1)-pos_pulses)*10/8000.0/1000.0) # calculates the meters from the pulses
              if pos_temp < 10: # if the meters position data is under 10 (aka correct)
                 return pos_temp       #  a check to ensure position isn't read incorrectly. Occassionally happens at the beginning of a move
           except ValueError:
              print 'ValueError' # if the pos_temp gives an incorrect or impossible value, print Value Error so the user knows

        return 0.0 # returns the var pos_meters


    def move_meters(self, position):
        pulses = int(position*1000*8000/self.lead)
        return self.move_pulses(pulses)

    def move_pulses(self, pulses):
        hex_position = hex(16**8 - 1 - int(round(pulses))).upper()[2:10]
        msg = self.axis_str + 'a' + hex_position + 2*self.zero # creates message
        self.serial_read_write(msg)
        return True

    # these handlers vary depending on if the input is pulses or meters
    def handle_move_pulses(self, req):
        #return MovePulsesResponse(abs_move(req.pulses, 0)) # calls to abs_pos
        return self.move_pulses(req.pulses)

    def handle_move_meters(self, req):
        #pulses = int(req.meters*1000*8000/lead) # converts desired meters position to pulses for abs_pos function
        #return MoveMetersResponse(abs_move(pulses, 1)) # calls to abs_pos
        return self.move_meters(req.meters)

    # current position publisher
    def talker(self):

        rate = rospy.Rate(5)                #  5hz
        while not rospy.is_shutdown():
            pos_meters = self.pos_inq()
            if self.parameter_lock.acquire():
                self.position_meters = pos_meters
                self.position_pub.publish(pos_meters) #  publishes pos_meters
                self.parameter_lock.release()
            rate.sleep()

#  all functions and services initializations
def services_init():
    rospy.init_node('robo_cylinder_services')

    my_driver = RoboCylinder()

    rospy.spin() #  wait for everything to be done

if __name__ == "__main__":
    services_init() #  calls the service_init function if the name is main
