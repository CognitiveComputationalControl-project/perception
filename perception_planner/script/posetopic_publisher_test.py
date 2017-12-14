#!/usr/bin/env python

import roslib; 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8
from perception_msgs.msg import Door_States
from std_msgs.msg import Bool
import std_msgs.msg
import geometry_msgs.msg as gm           
import move_base_msgs.msg as mbm   
import rospy
import math

topic = 'detected_handle_pos'
publisher = rospy.Publisher(topic, PoseStamped,queue_size=10)
rospy.init_node('publisher_test')
recog_pos=PoseStamped()
count = 0

while not rospy.is_shutdown():

  target_pose_Msg = rospy.wait_for_message("/detected_handle_pos_condition", PoseStamped)
  recog_pos.pose.position.x=target_pose_Msg.pose.position.x
  recog_pos.pose.position.y=target_pose_Msg.pose.position.y
  recog_pos.pose.position.z=target_pose_Msg.pose.position.z

  Gmsg=PoseStamped()
  Gmsg.header.frame_id = 'map'
  Gmsg.header.stamp = rospy.Time(0)
  Gmsg.pose.position = recog_pos.pose.position
  Gmsg.pose.orientation = recog_pos.pose.orientation
  # Gmsg.pose.orientation = gm.Quaternion(x=-0.6239,
  #                   y=0.7606,
  #                   z=-0.178559,
  #                   w=-0.01674413,)
  # Gmsg.pose=True
  publisher.publish(Gmsg)
  rospy.sleep(0.1)
  # states_msg=Door_States()
  # pub.publish(int_msg) 

