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

# imports namespace
from robo_cylinder.srv import *

import sys
import rospy

def move_pulses_client(pos):
    rospy.wait_for_service('move_pulses') # wait for service call
    try:
        move_pulses = rospy.ServiceProxy('move_pulses', MovePulses) # tries calls service
        resp1 = move_pulses(pos)
        return resp1.csum # returns value from service call
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
def usage():
    return "%s [pos]"%sys.argv[0]

if __name__ == "__main__": #if name is main and sys.argv length is 2, pos will equal first char of sys.agrv
    if len(sys.argv) == 2:
        pos = int(sys.argv[1])
    else: #if not, exit
        print usage()
        sys.exit(1)
    print "Requesting move of %d pulses"%(pos)
    print "Checksum %s sent"%(move_pulses_client(pos))
