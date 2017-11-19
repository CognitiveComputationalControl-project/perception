#!/usr/bin/env python

import roslib; 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Int8
from perception_msgs.msg import Door_States
import std_msgs.msg
import rospy
import math

topic = 'FSM/door_states'
publisher = rospy.Publisher(topic, Door_States,queue_size=10)
rospy.init_node('publisher_test')

count = 0

while not rospy.is_shutdown():

   states_msg=Door_States()
   states_msg.header.stamp = rospy.Time.now()
   states_msg.header.frame_id = "/map"
   states_msg.state=1
   states_msg.angle=21.5
   states_msg.push_mode=1

   # markerArray.markers.append(marker)
   publisher.publish(states_msg)
   # pub.publish(int_msg)

   rospy.sleep(0.1)
