#!/usr/bin/env python
import sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import numpy as np
import math
from naoqi import ALProxy

# This code will return a variable called command. It will be used to control the robot to do coresseponding reaction when different body part of the robot has been touched. command=0 as initial, command=1 right arm hand is being touched, command=2 right arm middle, command=3 right arm up, command =4 front side.

#global variable definition
id_right_arm_up=(1,2,26,27,28,29,30,31,32,33,34,35,36,37,38,39)
id_right_arm_middle=(3,4,5,6,7,8,9,10,11)
id_right_arm_hand=(12,13,14,15,16,17,18,19,20,21,22,23,24,25)
#id_front=(40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71)
id_front=None
id_right_arm=None
value_range_front=None
value_range_right_arm=None
index_right_arm=None
index_front=None
id_front_touched=None
id_right_arm_touched=None
command=np.zeros((1,5))
command_list=np.array([0,1,2,3,4])
#nao parameter
#nao_ip="169.254.80.21"
port=9559
#nao_ip="192.168.0.1"
act=False
nao_ip="127.0.0.1"

def callback_ids_front(data):
	global id_front
	id_front=data.data
def callback_range_front(data):
	global value_range_front
	global index_front
	value_range_front=data.data
	index_front=[i for i,x in enumerate(value_range_front) if x >=0.1]


def callback_control_ids_right_arm(data):
	global id_right_arm
	id_right_arm=data.data
def callback_range_right_arm(data):
	global value_range_right_arm
	global index_right_arm
	value_range_right_arm=data.data
	index_right_arm=[i for i,x in enumerate(value_range_right_arm) if x >=0.1]


def return_touched_id():
	global id_front_touched
	global id_right_arm_touched

	if (not index_front==None):
		id_front_touched=np.zeros((1,len(index_front)))
		for i in range(0,len(index_front)):
			id_front_touched[0,i]=id_front[index_front[i]]
	else: 
		id_front_touched=None
	
	if (not index_right_arm==None):
		id_right_arm_touched=np.zeros((1,len(index_right_arm)))
		for i in range(0,len(index_right_arm)):
			id_right_arm_touched[0,i]=id_right_arm[index_right_arm[i]]
	else: 
		id_right_arm_touched=None
	if (not index_front==None):
		id_front_touched=id_front_touched[0]
		id_right_arm_touched=id_right_arm_touched[0]

def command_of_recation():
	global command
	global act
	return_touched_id()

	if not(id_front_touched==None):
		not_empty_tup_front= not(not id_front_touched.any())
	else: 
		not_empty_tup_front= False

	if not(id_right_arm_touched==None):
		not_empty_tup_right_arm= not(not id_right_arm_touched.any())
 	else:
		not_empty_tup_right_arm=False

	print not_empty_tup_front,not_empty_tup_right_arm

	if not_empty_tup_front and not not_empty_tup_right_arm:
		act=True
		command[0,4]=command_list[4]		

	elif not not_empty_tup_front and not_empty_tup_right_arm:
		act=True
		len_id_right_arm=len(id_right_arm_touched)
		for i in range(0,len_id_right_arm):
			if any(j==id_right_arm_touched[i] for j in id_right_arm_up):
				command[0,3]=command_list[3]
				act=True
			elif any(j==id_right_arm_touched[i] for j in id_right_arm_middle):
				command[0,2]=command_list[2]
				act=True
			elif any(j==id_right_arm_touched[i] for j in id_right_arm_hand):
				command[0,1]=command_list[1]
				act=True
			else:
				command=np.zeros((1,5))
				act=False

	elif not_empty_tup_front and not_empty_tup_right_arm:
		act=True
		command[0,4]=command_list[4]
		len_id_right_arm=len(id_right_arm_touched)
		for i in range(0,len_id_right_arm):
			if any(j==id_right_arm_touched[i] for j in id_right_arm_up):
				command[0,3]=command_list[3]
			elif any(j==id_right_arm_touched[i] for j in id_right_arm_middle):
				command[0,2]=command_list[2]
			elif any(j==id_right_arm_touched[i] for j in id_right_arm_hand):
				command[0,1]=command_list[1]
			else:
				command=np.zeros((1,5))
				act=False
	else:
		command=np.zeros((1,5))
		act=False

	return command


def nao_navigation_control():

	tts = ALProxy("ALTextToSpeech",nao_ip, port)
	if act==True:
		if command[0,1]==1:
			#tts.say("Right hand")
			rospy.loginfo("\n Right hand")
			while act==True:
				motion.post.moveTo(0,0.1,0)
				if act==False:
					break
		if command[0,2]==2:
			#tts.say("Right arm middle")
			rospy.loginfo("\n Right arm middle")
			while act==True:
				motion.post.moveTo(0,-0.1,0)
				if act==False:
					break

		if command[0,3]==3:
			#tts.say("Right arm upper")
			rospy.loginfo("\n Right arm upper")
			while act==True:
				motion.post.moveTo(0.1,0,0)
				if act==False:
					break
		
		if command[0,4]==4:
			#tts.say("Front side")
			rospy.loginfo("\n Front side")
			while act==True:
				motion.post.moveTo(-0.1,0,0)
				if act==False:
					break



if __name__ == '__main__':

	rospy.init_node('nao_navigation', anonymous=True)

	rospy.Subscriber("range_front",Float32MultiArray,callback_range_front)
	rospy.Subscriber("range_right_arm",Float32MultiArray,callback_range_right_arm)
	rospy.Subscriber("ids_front",Int32MultiArray,callback_ids_front)	
	rospy.Subscriber("ids_right_arm",Int32MultiArray,callback_control_ids_right_arm)		
	

	rate = rospy.Rate(10)
	#nao initialization
	motion = ALProxy("ALMotion",nao_ip,port)
	motion.wakeUp()
	posture = ALProxy("ALRobotPosture",nao_ip,port)
	posture.goToPosture("StandInit",1.0)
	time.sleep(2)

	while not rospy.is_shutdown():

		command_of_recation()
		print command
		time.sleep(0.1)
		nao_navigation_control()
		








	
