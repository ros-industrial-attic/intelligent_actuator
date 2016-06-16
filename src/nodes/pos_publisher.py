#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import sys
import time

from std_msgs.msg import Float32

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

#global start and end characters for each checksum
stx = "02".decode("hex")
etx = "03".decode("hex")

#input parameters specific to this robo cylinder
#axis can be changed to 0-15 but ours operates on axis 0. Lead screw is supposedly 10mm
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

#Position inquiry
def pos_inq():
    msg = axis_str + 'R4' + 4*zero + '74' + 3*zero
    bcc_int = axis_val + ord('R') + 2*ord('4') + ord('7') + 7*ord(zero)
    bcc = bcc_calc(bcc_int)
    csum = stx + msg + bcc + etx
    
    #serial porty is touchy and will occassionally require flushing
    ser.flushInput()
    ser.flushOutput()
    ser.flush()
    ser.write(csum)
    time.sleep(.1)

    response = ''
    response += ser.read(size=16)
    pos_pulses = int(response[5:13], 16)
    
    #convert from encoder pulses to meters
    pos_meters = float(((16**8-1)-pos_pulses)*10/8000.0/1000.0)
    
    return pos_meters

def talker():
    pub = rospy.Publisher('/car/pos', Float32, queue_size=10)
    rospy.init_node('car_pos_publisher', anonymous=True)
    rate = rospy.Rate(1) 		# 1hz. Any faster will interfere with responses
    while not rospy.is_shutdown():
        pos = pos_inq()
        rospy.loginfo(pos)
        pub.publish(pos)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
