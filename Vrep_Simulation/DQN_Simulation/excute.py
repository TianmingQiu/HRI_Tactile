# -*- coding: utf-8 -*-
"""
@author: Pierre Jacquot
"""
#For more informations please check : http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm
import vrep,sys
from naoqi import ALProxy
import time
from manage_joints import get_first_handles,JointControl

from DQN import DQN
from ENV import ENV
import rospy
import time
import tensorflow as tf
import numpy as np
import random
from collections import deque
import roslib; roslib.load_manifest('numpy_tutorial')
from std_msgs.msg import Float32MultiArray
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32
import math


def RobotInit(motion):
	names = ["Body"]
	angles = [-0.038392066955566406, 0.1349501609802246, 1.1964781284332275, 0.07512402534484863, -1.4926238059997559, -1.3391400575637817, 0.11500811576843262, 0.029999971389770508, -0.25766992568969727, -0.09506607055664062, -0.9694461822509766, 2.086198091506958, -1.168950080871582, 0.07367396354675293, -0.25766992568969727, 0.10128593444824219, -0.9342479705810547, 2.0663399696350098, -1.186300277709961, -0.07205605506896973, -0.309826135635376, 0.24233007431030273, 0.06131792068481445, 0.8544800281524658, 1.5983860492706299, 0.17799997329711914]
	fractionMaxSpeed  = 0.1
	time.sleep(1)
	motion.setAngles(names, angles, fractionMaxSpeed)

def RobotStartUp(motion):
	print "Ready to sit down?[y/N]"
	keyboard_in = raw_input()
	if keyboard_in == 'y':
		pass
	motion.wakeUp()
	# posture.goToPosture("Crouch", 1.0)
	motion.waitUntilMoveIsFinished()
	RobotInit(motion)
	time.sleep(5)
	motion.openHand('RHand')
	time.sleep(2)
	motion.closeHand('RHand')
	time.sleep(1)

# ------------------------------------
# Hyper Parameters for DQN
GAMMA = 0.9 # discount factor for target Q
INITIAL_EPSILON = 0.5 # starting value of epsilon
FINAL_EPSILON = 0.01 # final value of epsilon
REPLAY_SIZE = 100000 # experience replay buffer size
BATCH_SIZE = 30 # size of minibatch

# ---------------------------------------------------------
# Hyper Parameters
EPISODE = 100 # Episode limitation
STEP = 20 # Step limitation in an episode
TEST = 10 # The number of experiment test every 100 episode


def main():
	print '================ Program Sarted ================'

	vrep.simxFinish(-1)
	clientID = vrep.simxStart('127.0.0.2', 19999, True, True, 5000, 5)
	if clientID != -1:
		print 'Connected to remote API server'

	else:
		print 'Connection non successful'
		sys.exit('Could not connect')

	print "================ Choregraphe's Initialization ================"
	print 'Enter your NAO IP'
	naoIP = raw_input()
	# naoIP = map(str,naoIP.split())
	print 'Enter your NAO port'
	naoPort = raw_input()
	naoPort = int(naoPort)
	# naoPort = map(int,naoPort.split())

	motion = ALProxy("ALMotion", naoIP, naoPort)
	posture = ALProxy("ALRobotPosture", naoIP, naoPort)

	# Go to the posture StandInitZero

	motion.stiffnessInterpolation('Body', 1.0, 1.0)
	print "stiffness set"
	posture.goToPosture("StandZero", 1.0)

	print "init pose stand zero"

	# postureProxy.goToPosture('StandZero', 1.0)

	Head_Yaw = [];
	Head_Pitch = [];
	L_Hip_Yaw_Pitch = [];
	L_Hip_Roll = [];
	L_Hip_Pitch = [];
	L_Knee_Pitch = [];
	L_Ankle_Pitch = [];
	L_Ankle_Roll = [];
	R_Hip_Yaw_Pitch = [];
	R_Hip_Roll = [];
	R_Hip_Pitch = [];
	R_Knee_Pitch = [];
	R_Ankle_Pitch = [];
	R_Ankle_Roll = [];
	L_Shoulder_Pitch = [];
	L_Shoulder_Roll = [];
	L_Elbow_Yaw = [];
	L_Elbow_Roll = [];
	L_Wrist_Yaw = []
	R_Shoulder_Pitch = [];
	R_Shoulder_Roll = [];
	R_Elbow_Yaw = [];
	R_Elbow_Roll = [];
	R_Wrist_Yaw = []
	R_H = [];
	L_H = [];
	R_Hand = [];
	L_Hand = [];
	Body = [Head_Yaw, Head_Pitch, L_Hip_Yaw_Pitch, L_Hip_Roll, L_Hip_Pitch, L_Knee_Pitch, L_Ankle_Pitch, L_Ankle_Roll,
			R_Hip_Yaw_Pitch, R_Hip_Roll, R_Hip_Pitch, R_Knee_Pitch, R_Ankle_Pitch, R_Ankle_Roll, L_Shoulder_Pitch,
			L_Shoulder_Roll, L_Elbow_Yaw, L_Elbow_Roll, L_Wrist_Yaw, R_Shoulder_Pitch, R_Shoulder_Roll, R_Elbow_Yaw,
			R_Elbow_Roll, R_Wrist_Yaw, L_H, L_Hand, R_H, R_Hand]

	get_first_handles(clientID, Body)
	print "================ Handles Initialization ================"
	commandAngles = motion.getAngles('Body', False)
	time.sleep(1)
	posture.goToPosture("StandZero", 1.0)
	posture.goToPosture("StandInit", 1.0)
	print "Stand Init"
	posture.goToPosture("Sit", 1.0)
	print "pose sit"
	# initialize OpenAI Gym env and dqn agent
	RobotStartUp(motion)
	env = ENV(motion)
	agent = DQN(env)
	print "Ready to train?[y/N]"
	keyboard_in = raw_input()
	if (keyboard_in == 'y'):
		pass

	for episode in xrange(EPISODE):
		print "Train"
		# initialize task
		joints = env.getJoint()
		time.sleep(0.2)
		joints = env.getJoint()
		state = env.calState(joints)
		guide = 0

		for step in xrange(STEP):
			print "Episode: %s, Step: %s" % (episode, step)

			# ------------------guide---------------------
			if guide != 0:
				print "Give a guide:"
				action_guide = raw_input()  # a very direct guide which should be changed later as a skin feedback
				# which to select a relating direction of action
				if (action_guide == '0') or (action_guide == '1') or (action_guide == '2') or (action_guide == '3'):
					pass
				else:
					print "Give a guide again:"
					action_guide = raw_input()
				action = str(action_guide)
			else:
				action = agent.egreedy_action(state)  # e-greedy action for train

			action = agent.egreedy_action(state)
			joints = env.getJoint()
			time.sleep(0.2)
			joints = env.getJoint()
			done = env.ActPerfm(action, joints)

			joints = env.getJoint()
			time.sleep(0.3)
			joints = env.getJoint()
			next_state = env.calState(joints)

			print "Skin will collect reward: good: 9 | bad: 0 "
			keyboard_in = raw_input()
			if (keyboard_in == '0'):
				reward = -10
				guide = 1
			else:
				reward = 10
				guide = 0
			print "reward already collected!"
			time.sleep(0.2)
			agent.perceive(state, int(action), reward, next_state, done)
			state = next_state
			if done:
				RobotInit()  # everytime when a episode is finishing, go back to initial position
				break


if __name__ == '__main__':
	rospy.init_node('central_node', anonymous=False)
	main()
