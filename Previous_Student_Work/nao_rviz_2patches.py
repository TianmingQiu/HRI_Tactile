#!/usr/bin/env python
import sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
from naoqi import ALProxy


#global variable definition
left = False
right = False
schalter = False
port = 9559
# When NAO is connected with the router:
nao_ip = "192.168.0.101"
# For RViz Simulation:
# nao_ip = "127.0.0.1"


def callback_range_left_arm_upper(data):
	global left
	global schalter
	left = any(j >= 0.1 for j in data.data)
	schalter = left or right


def callback_range_right_arm_upper(data):
	global right
	global schalter
	right = any(j >= 0.1 for j in data.data)
	schalter = left or right
	#print left, right, schalter


def nao_control():
	if left and right:
		#tts.say("I confuse where to go")
		rospy.loginfo("\n Stand")
		#posture.goToPosture("StandInit",1.0)
	elif left and not right:
		#tts.say("go right")
		rospy.loginfo("\n Go right")
		while left:
			motion.post.moveTo(0,-0.03,0)
			#if not schalter or right:
			#	motion.waitUntilMoveIsFinished()
			#	break
			if not schalter:
				motion.waitUntilMoveIsFinished()
				break
			elif right:
				motion.killMove()
				break
			time.sleep(0.2)
	elif not left and right:
		#tts.say("go left")
		rospy.loginfo("\n Go left")
		while right:
			motion.post.moveTo(0,0.03,0)
			if not schalter:
				motion.waitUntilMoveIsFinished()
				break
			elif left:
				motion.killMove()
				break
			time.sleep(0.2)
	else:
		rospy.loginfo("\n Stand")
		#posture.goToPosture("StandInit",1.0)
	return


if __name__ == '__main__':

	rospy.init_node('nao_navigation', anonymous=True)
	# Right arm:
	rospy.Subscriber("range_right_arm_upper",Float32MultiArray,callback_range_right_arm_upper)
	# Left arm:
	rospy.Subscriber("range_left_arm_upper",Float32MultiArray,callback_range_left_arm_upper)

	motion = ALProxy("ALMotion",nao_ip,port)
	motion.wakeUp()
	posture = ALProxy("ALRobotPosture",nao_ip,port)
	#posture.goToPosture("StandInit",1.0)
	time.sleep(2)
	
	while not rospy.is_shutdown():
		time.sleep(0.1)
		if left or right:
			nao_control()

