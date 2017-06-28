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

import roslib
import rospy

import tf
from std_msgs.msg import Float32

def handle_car_pos(msg, car_link):
    br = tf.TransformBroadcaster() # calls transform broadcast
    base_link = rospy.get_param('~robo_base') # gets base link value from launch file \
    # sends transform information
    br.sendTransform((msg.data, 0, 0),
                     (0, 0, 0, 0),
                     rospy.Time.now(),
                     car_link,
                     base_link)

if __name__ == '__main__': # if the name is main
    rospy.init_node('car_tf_broadcaster') #initialize this node
    car_link = "car_link" #establish car link
    # subscribes to handle_car publisher
    rospy.Subscriber('/car/pos',
                     Float32,
                     handle_car_pos,
                     car_link)
    rospy.spin() # wait 
