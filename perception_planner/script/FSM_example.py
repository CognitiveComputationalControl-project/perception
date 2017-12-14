#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import pylab as pl
import trajectory_msgs.msg
from perception_msgs.msg import Door_States
from random import randint
from time import clock

State =type("State", (object,),{})

class Open(State):
	def Execute(self):
		print "Door is Fully Open"

class Closed(State):
	def Execute(self):
		print "Door is Fully Closed"

class HalfOpen(State):
	def Execute(self):
		print "Door is Half Open"

class Transition(object):
	def __init__(self,toState):
		self.toState=toState

	def Execute(self):
		print "Transitioning"

class SimpleFSM(object):
	def __init__(self,char):
		self.char=char
		self.states={}
		self.transitions={}
		self.curState=None
		self.trans=None

	def SetState(self, stateName):
		self.curState=self.states[stateName]

	def Transition(self,transName):
		self.trans=self.transitions[transName]

	def Execute(self):
		if(self.trans):
			self.trans.Execute()
			self.SetState(self.trans.toState)
			self.trans=None
		self.curState.Execute()

class Flag(object):
	def __init__(self):
		self.init_start=True
		self.state_active=False

class Char(object):
	def __init__(self):
		self.FSM=SimpleFSM(self)
		self.FullOpen = True


Door = Char()
Door.FSM.states["Open"]=Open()
Door.FSM.states["Closed"]=Closed()
Door.FSM.states["HalfOpen"]=HalfOpen()
Door.FSM.transitions["toOpen"]=Transition("Open")
Door.FSM.transitions["toHalfOpen"]=Transition("HalfOpen")
Door.FSM.transitions["toClosed"]=Transition("Closed")
Door.FSM.transitions["toNothing"]=Transition("Nothing")
Door.FSM.SetState("Closed")

startTime=0
CurTime=0
state_active=False

Cur_flag= Flag()

def door_state_callback(msg):
	# print msg.state
	global startTime

	# measuring time from the activation timing
	if (Cur_flag.init_start):
		startTime=rospy.Time.now().secs
		Cur_flag.init_start=False
	else:
		cur_time = rospy.Time.now().secs
		time_diff=cur_time-startTime
		print time_diff
	
	# change the active mode if time pass 10 seconds
	if (time_diff>10):
		Cur_flag.state_active=True
	else:
		pass

	# Finite State Machine
	if(Cur_flag.state_active):
		if(msg.state==1):
			print "Dor is closed"
			Door.FSM.Transition("toOpen")

		elif(msg.state==2):
			print "Dor is partially-Open"
			print msg.angle
			Door.FSM.Transition("toOpen")
			
		else:
			print "Dor is fully open"
			Door.FSM.Transition("Nothing")
		
		startTime=rospy.Time.now().secs
		Cur_flag.state_active=False
	else:
		print "wait for time"

if __name__=="__main__":
	rospy.init_node('Finite_State_Machine')
	rospy.Subscriber("FSM/door_states",Door_States, door_state_callback)

	rospy.spin()
