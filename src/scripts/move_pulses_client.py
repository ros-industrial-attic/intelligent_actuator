#!/usr/bin/env python

from robo_cylinder.srv import *

import sys
import rospy

def move_pulses_client(pos):
    rospy.wait_for_service('move_pulses')
    try:
        move_pulses = rospy.ServiceProxy('move_pulses', MovePulses)
        resp1 = move_pulses(pos)
        return resp1.csum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
def usage():
    return "%s [pos]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        pos = int(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Requesting move of %d pulses"%(pos)
    print "Checksum %s sent"%(move_pulses_client(pos))
