#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import pylab as pl
import trajectory_msgs.msg
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

class Char(object):
	def __init__(self):
		self.FSM=SimpleFSM(self)
		self.FullOpen = True

if __name__=="__main__":
	rospy.init_node('Finite_State_Machine')
	Door = Char()

	Door.FSM.states["Open"]=Open()
	Door.FSM.states["Closed"]=Closed()
	Door.FSM.states["HalfOpen"]=HalfOpen()
	Door.FSM.transitions["toOpen"]=Transition("Open")
	Door.FSM.transitions["toHalfOpen"]=Transition("HalfOpen")
	Door.FSM.transitions["toClosed"]=Transition("Closed")
	Door.FSM.SetState("Closed")

	rospy.spin()

	for i in range(20):
		startTime =clock()
		timeInterval=1
		while(startTime+timeInterval > clock()):
			pass
		if(randint(0,2)):
			if(Door.FullOpen):
				Door.FSM.Transition("toHalfOpen")
				Door.FullOpen=False
			else:
				Door.FSM.Transition("toOpen")
				Door.FullOpen=True
		Door.FSM.Execute()






















