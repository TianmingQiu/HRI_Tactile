#!/usr/bin/env python
import sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
from naoqi import ALProxy
port=9559
#nao_ip="192.168.0.1"
nao_ip="127.0.0.1"

#global variable definition
left=0
right=0
schalter=False

def callback_range_left_arm_upper(data):
	global left
	left=any(j>=0.1 for j in data.data)
def callback_range_right_arm_upper(data):
	global right
	right=any(j>=0.1 for j in data.data)
	
def schalter_determine():
	global schalter
	schalter= left or right
	print left, right, schalter
def nao_control():
	if left and right:
		#tts.say("I confuse where to go")
		rospy.loginfo("\n Stand")
		posture.goToPosture("StandInit",1.0)
	elif left and not right:
		#tts.say("go right")
		rospy.loginfo("\n go right")
		while True:
			motion.post.moveTo(0,0.1,0)
			if schalter==False:
				motion.killMove()
				#motion.waitUntilMoveIsFinished()
				break
	elif not left and right:
		#tts.say("go left")
		rospy.loginfo("\n go left")
		while True:
			motion.post.moveTo(0,-0.1,0)
			if schalter==False:
				motion.killMove()
				#motion.waitUntilMoveIsFinished()
				break
	else:
		#tts.say("I will stay here")
		rospy.loginfo("\n Stand")
		posture.goToPosture("StandInit",1.0)



if __name__ == '__main__':

	rospy.init_node('nao_navigation', anonymous=True)
	rospy.Subscriber("range_right_arm_upper",Float32MultiArray,callback_range_right_arm_upper)
	rospy.Subscriber("range_left_arm_upper",Float32MultiArray,callback_range_left_arm_upper)

	motion = ALProxy("ALMotion",nao_ip,port)
	motion.wakeUp()
	posture = ALProxy("ALRobotPosture",nao_ip,port)
	posture.goToPosture("StandInit",1.0)
	time.sleep(2)
	
	while not rospy.is_shutdown():
		schalter_determine()
		time.sleep(0.1)
		#print left, right
		if schalter== True:
			nao_control()

