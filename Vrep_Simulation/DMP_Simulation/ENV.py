import rospy
import sys
from naoqi import ALProxy
import time
import numpy as np 
import random
from collections import deque
import roslib
from std_msgs.msg import Float32MultiArray
from rospy_tutorials.msg import Floats
from std_msgs.msg import Int32
import math
from manage_joints import get_first_handles,JointControl,JointControlFull

# ------------------------------------
# Hyper Parameters for DQN
GAMMA = 0.9 # discount factor for target Q
INITIAL_EPSILON = 0.5 # starting value of epsilon
FINAL_EPSILON = 0.01 # final value of epsilon
REPLAY_SIZE = 100000 # experience replay buffer size
BATCH_SIZE = 30 # size of minibatch
STEP_SIZE = 0.05


class ENV(object):
    def __init__(self, robot_cfg):
        self.reward = 0
        self.state = 1
        self.guide = 0
        self.joint = (0.24846601486206055, 0.8529460430145264)
        self.state_dim = 2
        self.action_dim = 4
        self.motion = robot_cfg[0]
        self.clientID = robot_cfg[1]
        self.body = robot_cfg[2]
        self.time_interval = 0.2

    def pose(self, th1,th2):
        whole_angles = self.motion.getAngles('Body', False)
        whole_angles[21] = th1
        whole_angles[23] = th2
        self.motion.setAngles('Body', whole_angles, 0.1)
        time.sleep(self.time_interval)
        JointControlFull(self.clientID, self.motion, 0, self.body)
        print "env act"


    def robot_init(self):
        angles = [-0.038392066955566406, 0.1349501609802246, 1.1964781284332275, 0.07512402534484863,
                  -1.4926238059997559, -1.3391400575637817, 0.11500811576843262, 0.029999971389770508,
                  -0.25766992568969727, -0.09506607055664062, -0.9694461822509766, 2.086198091506958,
                  -1.168950080871582, 0.07367396354675293, -0.25766992568969727, 0.10128593444824219,
                  -0.9342479705810547, 2.0663399696350098, -1.186300277709961, -0.07205605506896973, -0.309826135635376,
                  -0.24233007431030273, 0.06131792068481445, 0.8544800281524658, 1.5983860492706299, 0.17799997329711914]

        # original shoulder angles[21] = 0.2433007431030273

        self.motion.setAngles('Body', angles, 0.1)
        time.sleep(3)
        JointControlFull(self.clientID, self.motion, 0, self.body)





