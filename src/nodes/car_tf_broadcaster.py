#!/usr/bin/env python  
import roslib
import rospy

import tf
from std_msgs.msg import Float32

def handle_car_pos(msg, car_link):
    br = tf.TransformBroadcaster()
    base_link = rospy.get_param('~robo_base')
    br.sendTransform((msg.data, 0, 0),
                     (0, 0, 0, 0),
                     rospy.Time.now(),
                     car_link,
                     base_link)

if __name__ == '__main__':
    rospy.init_node('car_tf_broadcaster')
    car_link = "car_link"
    rospy.Subscriber('/car/pos',
                     Float32,
                     handle_car_pos,
                     car_link)
    rospy.spin()
