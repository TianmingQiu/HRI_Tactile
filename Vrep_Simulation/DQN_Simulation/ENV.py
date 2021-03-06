import rospy
import sys
from naoqi import ALProxy
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

    def cal_ave_reward(self): # ToDO
        rospy.Subscriber('chatter', Int32, self.reward_CallBack)
        return self.reward

    def calReward(self): # ToDO
        rospy.Subscriber("force_hand", Float32MultiArray, self.reward_CB)
        return self.reward, self.guide

    def reward_CallBack(self, data):
        self.reward = data

    def reward_CB(self, data):
        cell_sum = 0
        # calculate reward according to skin cell with different weightings
        for i in range(14):
            cell_sum = cell_sum + data.data[i]
        #self.reward = - (self.reward + 5 * (data.data[0] + data.data[1] + data.data[9] + data.data[11]))

        # below need to be tested:
        part_out = data.data[0] + data.data[1] + data.data[13] + data.data[11]
        part_in = (cell_sum - part_out) / 8
        part_out = (part_out - data.data[0] - data.data[13])/ 2
        if part_in < 0.008:
            part_in = 0
        else:
            part_in = part_in - 0.008
        if part_out < 0.008:
            part_out = 0
        else:
            part_out = part_out - 0.008
        deviation = part_out -  part_in
        #print deviation
        if part_in > 0.02:
            self.guide = -1
        elif part_out > 0.02:
            self.guide = 1
        else:
             self.guide = 0

        reward0 = 10 - (part_out + part_in) * (part_out + part_in) * 10000
        reward_result = (reward0 + 200) / 400
        self.reward = reward_result

    def getJoint(self):
        whole_angles = self.motion.getAngles('Body', False)
        self.joint = (whole_angles[21], whole_angles[23])
        return self.joint

    def ActPerfm(self, act_cmd, joint):
        print "Action Command: %s" %act_cmd
        target_deviation = (joint[0] + 1.0768810510635376, joint[1] - 1.2311931848526)
        
        IsSafe = (joint[0] <= 0.32) and (joint[0] >= -1.3) and (joint[1] <= 1.5) and (joint[1] >= 0.03) #this range is wrong
        #IsSafe = True
        if IsSafe:

            if (target_deviation[0] < 0.02) and (target_deviation[1] < 0.02):
                return True
            else:
                fun = {
                    '0': self.ShoulderIn,
                    '1': self.ShoulderOut,
                    '2': self.ElbowIn,
                    '3': self.ElbowOut,
                }[act_cmd]
                fun()
                return False
            
        else:
            return True

    def ShoulderIn(self):
        whole_angles = self.motion.getAngles('Body', False)
        whole_angles[21] += STEP_SIZE
        self.motion.setAngles('Body', whole_angles, 0.1)
        time.sleep(2)
        JointControlFull(self.clientID, self.motion, 0, self.body)
        print "shoulder in"

    def ShoulderOut(self):
        whole_angles = self.motion.getAngles('Body', False)
        whole_angles[21] -= STEP_SIZE
        self.motion.setAngles('Body', whole_angles, 0.1)
        time.sleep(2)
        JointControlFull(self.clientID, self.motion, 0, self.body)
        print "shoulder out"

    def ElbowIn(self):
        whole_angles = self.motion.getAngles('Body', False)
        whole_angles[23] += STEP_SIZE
        self.motion.setAngles('Body', whole_angles, 0.1)
        time.sleep(2)
        JointControlFull(self.clientID, self.motion, 0, self.body)
        print "elbow in"

    def ElbowOut(self):
        whole_angles = self.motion.getAngles('Body', False)
        whole_angles[23] -= STEP_SIZE
        self.motion.setAngles('Body', whole_angles, 0.1)
        time.sleep(2)
        JointControlFull(self.clientID, self.motion, 0, self.body)
        print "elbow out"

    def calState(self, joint):
        a0 = 0.05 * int(round(joint[0] / 0.05))
        b0 = 0.05 * int(round(joint[1] / 0.05))

        # normalization
        a = (a0 + 1.3265) / 1.6407
        b = (b0 - 0.0349) / 1.5097
        
        state_result = [a, b]
        self.state = np.array(state_result)
        return self.state

    def robot_init(self):
        angles = [-0.038392066955566406, 0.1349501609802246, 1.1964781284332275, 0.07512402534484863,
                  -1.4926238059997559, -1.3391400575637817, 0.11500811576843262, 0.029999971389770508,
                  -0.25766992568969727, -0.09506607055664062, -0.9694461822509766, 2.086198091506958,
                  -1.168950080871582, 0.07367396354675293, -0.25766992568969727, 0.10128593444824219,
                  -0.9342479705810547, 2.0663399696350098, -1.186300277709961, -0.07205605506896973, -0.309826135635376,
                  0.24233007431030273, 0.06131792068481445, 0.8544800281524658, 1.5983860492706299, 0.17799997329711914]

        self.motion.setAngles('Body', angles, 0.1)
        time.sleep(3)
        JointControlFull(self.clientID, self.motion, 0, self.body)





