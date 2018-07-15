#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import numpy as np
import math
from naoqi import ALProxy
import time

temp_range=None
#nao_ip="169.254.80.21"
port=9559
nao_ip="127.0.0.1"
#port=22
ids_array =None
index=None
schalter=False

### This part subscribes topics and push the messages' data into an array###
def callback_range(data):
	global temp_range
	global schalter
	temp_range=data.data
	#print temp_range
	if any(j>=0.1 for j in temp_range):
		schalter=True
	else:
		schalter=False
	
def nao_action():
	global index
	tts = ALProxy("ALTextToSpeech",nao_ip, 9559)
	index=[i for i,x in enumerate(temp_range) if x >=0.1]
	input=ids_array[index[0]]
	print input
	if input==1:
		tts.say("Forward")
		rospy.loginfo("\n Forward")
		while counter>0:
			motion.post.moveTo(0.1,0,0)
			if schalter==False:
				#motion.post.moveTo(0.025,0,0)
				counter=0
	if input==2:
		tts.say("Backward")
		rospy.loginfo("\n Backward")
		while counter>0:
			motion.post.moveTo(-0.1,0,0)
			if schalter==False:
				#motion.post.moveTo(0.025,0,0)
				counter=0
	if input==3:
		tts.say("Left")
		rospy.loginfo("\n Left")
		while counter>0:
			motion.post.moveTo(0,0.1,0)
			if schalter==False:
				#motion.post.moveTo(0.025,0,0)
				counter=0
	if input==4:
		tts.say("Right")
		rospy.loginfo("\n Right")
		while counter>0:
			motion.post.moveTo(0,-0.1,0)
			if schalter==False:
				#motion.post.moveTo(0.025,0,0)
				counter=0
	#posture.goToPosture("StandInit",1.0)
def skincell_control_ids(data):
	global ids_array
	ids_array=data.data

if __name__ == '__main__':

	rospy.init_node('tactile_identifiation', anonymous=True)
	rospy.Subscriber("range_topic",Float32MultiArray,callback_range)
	rospy.Subscriber("ids",Int32MultiArray,skincell_control_ids)

	#nao initialization
	motion = ALProxy("ALMotion",nao_ip,port)
	motion.wakeUp()
	posture = ALProxy("ALRobotPosture",nao_ip,port)
	posture.goToPosture("StandInit",1.0)
	time.sleep(2)
	
	while not rospy.is_shutdown():
		#print temp_range,index,ids_array
		if schalter==True:
			nao_action()
			counter=1
			

		
