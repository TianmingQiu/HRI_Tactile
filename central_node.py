#!/usr/bin/env python
# -------------------------------
# DQN for NAO and skin
# Author: Tianming Qiu
# Date: 2017.Sep.29
# All rights reserved
# -------------------------------

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
import math

nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)


def RobotInit():
    motion.wakeUp()
    posture.goToPosture("Crouch", 1.0)
    motion.waitUntilMoveIsFinished()
    names  = ["Body"]
    angles  = [-0.038392066955566406, 0.1349501609802246, 1.1964781284332275, 0.07512402534484863, -1.4926238059997559, -1.3391400575637817, 0.11500811576843262, 0.029999971389770508, -0.25766992568969727, -0.09506607055664062, -0.9694461822509766, 2.086198091506958, -1.168950080871582, 0.07367396354675293, -0.25766992568969727, 0.10128593444824219, -0.9342479705810547, 2.0663399696350098, -1.186300277709961, -0.07205605506896973, -0.309826135635376, 0.24233007431030273, 0.06131792068481445, 0.8544800281524658, 1.5983860492706299, 0.17799997329711914]
    fractionMaxSpeed  = 0.1
    time.sleep(1)
    motion.setAngles(names, angles, fractionMaxSpeed)

    time.sleep(5)
    motion.openHand('RHand')
    time.sleep(2)
    motion.closeHand('RHand')
    time.sleep(1)
    print "Ready!"


class ENV():
    def __init__(self):
        self.reward = 0
        self.state = 1
        self.joint = (0,0)
        self.state_dim = 2
        self.action_dim = 4


    def calReward(self):
        rospy.Subscriber("force_hand", Float32MultiArray, self.reward_CB)
        return self.reward
    def reward_CB(self, data):
        
        self.reward = 0
        # calculate reward according to skin cell with different weightings
        for i in range(14):
            self.reward = self.reward + data.data[i]
        #self.reward = - (self.reward + 5 * (data.data[0] + data.data[1] + data.data[9] + data.data[11]))
        self.reward = - self.reward
        


    def getJoint(self):
        rospy.Subscriber("floats", Floats, self.joint_CB)
        return self.joint
    def joint_CB(self, data):
        self.joint = data.data

    def ActPerfm(self, act_cmd, joint):
        #print "input: %s, %s" %(act_cmd, joint)
        IsSafe = (joint[0] < 0.3) and (joint[0] > -1.3) and (joint[1] < 1.5) and (joint[1] > 0.035) #this range is wrong
        #IsSafe = True
        if IsSafe:
            print "Done?"
            keyboard_in = raw_input()
            if (keyboard_in == '1'):
                return True
            else:
                fun = {
                    '0': self.ShoulderF,
                    '1': self.ShoulderB,
                    '2': self.ElbowF,
                    '3': self.ElbowB,
                }[act_cmd]
                fun(joint)
                return False
            
        else:
            return True

    def ShoulderF(self, joint):
        
        new_angle = joint[0] + 0.1
        motion.setAngles("RShoulderRoll", new_angle, 0.2)
        #time.sleep(1)
        

    def ShoulderB(self, joint):
        
        new_angle = joint[0] - 0.1
        motion.setAngles("RShoulderRoll", new_angle, 0.2)


    def ElbowF(self, joint):
        
        new_angle = joint[1] + 0.1
        motion.setAngles("RElbowRoll", new_angle, 0.2)
        #time.sleep(1)


    def ElbowB(self, joint):
        
        new_angle = joint[1] - 0.1
        motion.setAngles("RElbowRoll", new_angle, 0.2)
        #time.sleep(1)


    def calState(self, joint):
        a = round(self.joint[0], 1)
        b = round(self.joint[1], 1)
        #a0 = 0.211
        #b0 = 0.833
        # [0.21932005882263184, 0.8268680572509766]
        #state_result = int(8 * round((a0 - a) / 0.1) + round((b0 - b) / 0.1) + 1)
        state_result = [a, b]
        self.state = np.array(state_result)
        return self.state



# ------------------------------------
# Hyper Parameters for DQN
GAMMA = 0.9 # discount factor for target Q 
INITIAL_EPSILON = 1 # starting value of epsilon
FINAL_EPSILON = 0.01 # final value of epsilon
REPLAY_SIZE = 10000 # experience replay buffer size
BATCH_SIZE = 32 # size of minibatch

class DQN():
    # DQN Agent
    def __init__(self, env):
        # init experience replay
        self.replay_buffer = deque()
        # init some parameters
        self.time_step = 0
        self.epsilon = INITIAL_EPSILON
        self.state_dim = env.state_dim
        self.action_dim = env.action_dim

        self.create_Q_network()
        self.create_training_method()

        # Init session
        self.session = tf.InteractiveSession()
        self.session.run(tf.global_variables_initializer())

        # loading networks
        self.saver = tf.train.Saver()
        checkpoint = tf.train.get_checkpoint_state("saved_networks")
        if checkpoint and checkpoint.model_checkpoint_path:
                self.saver.restore(self.session, checkpoint.model_checkpoint_path)
                print "Successfully loaded:", checkpoint.model_checkpoint_path
        else:
                print "Could not find old network weights"

        global summary_writer
        summary_writer = tf.summary.FileWriter('~/logs',graph=self.session.graph)

    def create_Q_network(self):
        # network weights
        W1 = self.weight_variable([self.state_dim,20])
        b1 = self.bias_variable([20])
        W2 = self.weight_variable([20,self.action_dim])
        b2 = self.bias_variable([self.action_dim])
        # input layer
        self.state_input = tf.placeholder("float",[None,self.state_dim])
        # hidden layers
        h_layer = tf.nn.relu(tf.matmul(self.state_input,W1) + b1)
        # Q Value layer
        self.Q_value = tf.matmul(h_layer,W2) + b2


    def create_training_method(self):
        self.action_input = tf.placeholder("float",[None,self.action_dim]) # one hot presentation
        self.y_input = tf.placeholder("float",[None])
        Q_action = tf.reduce_sum(tf.multiply(self.Q_value,self.action_input),reduction_indices = 1)
        self.cost = tf.reduce_mean(tf.square(self.y_input - Q_action))
        tf.summary.scalar("loss",self.cost)
        global merged_summary_op
        merged_summary_op = tf.summary.merge_all()
        self.optimizer = tf.train.AdamOptimizer(0.0001).minimize(self.cost)

    def perceive(self,state,action,reward,next_state,done):
        one_hot_action = np.zeros(self.action_dim)
        one_hot_action[action] = 1
        self.replay_buffer.append((state,one_hot_action,reward,next_state,done))
        print "length of buffer:"
        print len(self.replay_buffer)
        if len(self.replay_buffer) > REPLAY_SIZE:
            self.replay_buffer.popleft()

        if len(self.replay_buffer) > BATCH_SIZE:
            self.train_Q_network()

    def train_Q_network(self):
        self.time_step += 1
        # Step 1: obtain random minibatch from replay memory
        minibatch = random.sample(self.replay_buffer,BATCH_SIZE)
        state_batch = [data[0] for data in minibatch]
        action_batch = [data[1] for data in minibatch]
        reward_batch = [data[2] for data in minibatch]
        next_state_batch = [data[3] for data in minibatch]

        # Step 2: calculate y
        y_batch = []
        Q_value_batch = self.Q_value.eval(feed_dict={self.state_input:next_state_batch})
        for i in range(0,BATCH_SIZE):
            done = minibatch[i][4]
            if done:
                y_batch.append(reward_batch[i])
            else :
                y_batch.append(reward_batch[i] + GAMMA * np.max(Q_value_batch[i]))

        self.optimizer.run(feed_dict={
            self.y_input:y_batch,
            self.action_input:action_batch,
            self.state_input:state_batch
            })
        summary_str = self.session.run(merged_summary_op,feed_dict={
                self.y_input : y_batch,
                self.action_input : action_batch,
                self.state_input : state_batch
                })
        summary_writer.add_summary(summary_str,self.time_step)

        # save network every 1000 iteration
        if self.time_step % 1000 == 0:
            self.saver.save(self.session, 'saved_networks/' + 'network' + '-dqn', global_step = self.time_step)

    def egreedy_action(self,state):
        Q_value = self.Q_value.eval(feed_dict = {
            self.state_input:[state]
            })[0]
        if random.random() <= self.epsilon:
            act_cmd = random.randint(0,self.action_dim - 1)
        else:
            act_cmd = np.argmax(Q_value)


        self.epsilon -= (INITIAL_EPSILON - FINAL_EPSILON)/10000

        return str(act_cmd)

    def action(self,state):
        act_cmd = np.argmax(self.Q_value.eval(feed_dict = {
            self.state_input:[state]
            })[0])
        return str(act_cmd)

    def weight_variable(self,shape):
        initial = tf.truncated_normal(shape)
        return tf.Variable(initial)

    def bias_variable(self,shape):
        initial = tf.constant(0.01, shape = shape)
        return tf.Variable(initial)

# ---------------------------------------------------------
# Hyper Parameters
EPISODE = 10000 # Episode limitation
STEP = 300 # Step limitation in an episode
TEST = 10 # The number of experiment test every 100 episode

def main():
    # initialize OpenAI Gym env and dqn agent
    RobotInit()
    env = ENV()
    agent = DQN(env)

    for episode in xrange(EPISODE):
        # initialize task
        joints = env.getJoint()
        time.sleep(0.2)
        joints = env.getJoint()
        state = env.calState(joints)
        #print joints
        # Train 
        for step in xrange(STEP):
            print "Episode: %s, Step: %s" %(episode, step)
            action = agent.egreedy_action(state) # e-greedy action for train
            
            done = env.ActPerfm(action, joints)
            joints = env.getJoint()
            time.sleep(0.2)
            joints = env.getJoint()
            next_state = env.calState(joints)
            print "Skin will collect reward!"
            time.sleep(4)
            reward = env.calReward()
            print "Finish collecting reward"
            print "action: %s, state: %s, reward: %s, done: %s" %(action,state,reward,done)
            time.sleep(1)
            # next_state,reward,done,_ = env.step(action) # these three results can be calculate independently
            # Define reward for agent
            # reward_agent = -1 if done else 0.1
            agent.perceive(state,int(action),reward,next_state,done)
            state = next_state
            if done:
                break
        # Test every 100 episodes
        if episode % 100 == 0:
            print " "
            print "TEST"
            total_reward = 0
            for i in xrange(TEST):
                joints = env.getJoint()
                time.sleep(0.2)
                joints = env.getJoint()
                state = env.calState(joints)
                for j in xrange(STEP):
                    #env.render()!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                    action = agent.action(state) # direct action for test
                    done = env.ActPerfm(action, joints)
                    joints = env.getJoint()
                    time.sleep(0.2)
                    joints = env.getJoint()
                    next_state = env.calState(joints)
                    print "Skin will collect reward!"
                    time.sleep(4)
                    reward = env.calReward()
                    print "Finish collecting reward"
                    print "action: %s, state: %s, reward: %s, done: %s" %(action,state,reward,done)
                    total_reward += reward
                    if done:
                        break
            ave_reward = total_reward/TEST
            print 'episode: ',episode,'Evaluation Average Reward:',ave_reward
            if ave_reward >= 200:
                break

    # save results for uploading
    #env.monitor.start('gym_results/CartPole-v0-experiment-1',force = True)
    for i in xrange(100):
        print " "
        print "Uploading!"
        joints = env.getJoint()
        time.sleep(0.2)
        joints = env.getJoint()
        state = env.calState(joints)
        for j in xrange(200):
            #env.render()
            action = agent.action(state) # direct action for test
            done = env.ActPerfm(action, joints)
            joints = env.getJoint()
            time.sleep(0.2)
            joints = env.getJoint()
            next_state = env.calState(joints)
            print "Skin will collect reward!"
            time.sleep(4)
            reward = env.calReward()
            print "Finish collecting reward"
            total_reward += reward
            if done:
                break
    # env.monitor.close() 

if __name__ == '__main__':
    rospy.init_node('central_node', anonymous = False)
    main()
    
