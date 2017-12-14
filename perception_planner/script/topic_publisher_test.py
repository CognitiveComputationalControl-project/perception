#!/usr/bin/env python

import roslib; 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int8
from perception_msgs.msg import Door_States
from std_msgs.msg import Bool
import std_msgs.msg
import rospy
import math

topic = 'is_localization'
publisher = rospy.Publisher(topic, Bool,queue_size=10)
rospy.init_node('publisher_test')

count = 0

while not rospy.is_shutdown():

   bool_msg=Bool()
   bool_msg.data=True
   publisher.publish(bool_msg)
   rospy.sleep(0.1)
   # states_msg=Door_States()
   # pub.publish(int_msg) 
