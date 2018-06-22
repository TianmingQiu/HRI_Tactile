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
from std_msgs.msg import Int32
import math

#nao_ip = "10.0.29.2"
nao_ip = "169.254.80.21"
#nao_ip = "127.0.0.1"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)
STEP_SIZE = 0.05

def RobotInit():
    names  = ["Body"]
    angles  = [-0.038392066955566406, 0.1349501609802246, 1.1964781284332275, 0.07512402534484863, -1.4926238059997559, -1.3391400575637817, 0.11500811576843262, 0.029999971389770508, -0.25766992568969727, -0.09506607055664062, -0.9694461822509766, 2.086198091506958, -1.168950080871582, 0.07367396354675293, -0.25766992568969727, 0.10128593444824219, -0.9342479705810547, 2.0663399696350098, -1.186300277709961, -0.07205605506896973, -0.309826135635376, 0.24233007431030273, 0.06131792068481445, 0.8544800281524658, 1.5983860492706299, 0.17799997329711914]
    fractionMaxSpeed  = 0.1
    time.sleep(1)
    motion.setAngles(names, angles, fractionMaxSpeed)

def RobotStartUp():
    motion.wakeUp()
    posture.goToPosture("Crouch", 1.0)
    motion.waitUntilMoveIsFinished()
    RobotInit()
    time.sleep(5)
    motion.openHand('RHand')
    time.sleep(2)
    motion.closeHand('RHand')
    time.sleep(1)



class ENV():
    def __init__(self):
        self.reward = 0
        self.state = 1
        self.guide = 0
        self.joint = (0.24846601486206055, 0.8529460430145264)
        self.state_dim = 2
        self.action_dim = 4


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
        rospy.Subscriber("floats", Floats, self.joint_CB)
        return self.joint


    def joint_CB(self, data):
        self.joint = data.data

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
                fun(joint)
                return False
            
        else:
            return True

    def RepeatPerfm(self, act_cmd, joint):
        fun = {
            '0': self.ShoulderIn,
            '1': self.ShoulderOut,
            '2': self.ElbowIn,
            '3': self.ElbowOut,
        }[act_cmd]
        fun(joint)


    def ShoulderIn(self, joint):
        new_angle = joint[0] + STEP_SIZE
        motion.setAngles("RShoulderRoll", new_angle, 0.2)
        print "0: ShoulderIn"


    def ShoulderOut(self, joint):
        new_angle = joint[0] - STEP_SIZE
        motion.setAngles("RShoulderRoll", new_angle, 0.2)
        print "1: ShoulderOut"


    def ElbowIn(self, joint):
        new_angle = joint[1] + STEP_SIZE
        motion.setAngles("RElbowRoll", new_angle, 0.2)
        print "2: ElbowIn"



    def ElbowOut(self, joint):
        new_angle = joint[1] - STEP_SIZE
        motion.setAngles("RElbowRoll", new_angle, 0.2)
        print "3: ElbowOut"


    def calState(self, joint):

        a0 = 0.05 * int(round(self.joint[0] / 0.05))
        b0 = 0.05 * int(round(self.joint[1] / 0.05))

        # normalization
        a = (a0 + 1.3265) / 1.6407
        b = (b0 - 0.0349) / 1.5097
        
        state_result = [a, b]
        self.state = np.array(state_result)
        return self.state



# ------------------------------------
# Hyper Parameters for DQN
GAMMA = 0.9 # discount factor for target Q 
INITIAL_EPSILON = 0.5 # starting value of epsilon
FINAL_EPSILON = 0.01 # final value of epsilon
REPLAY_SIZE = 100000 # experience replay buffer size
BATCH_SIZE = 30 # size of minibatch

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
        print "train Q network: %sth" %self.time_step
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

        # save network every 30 iteration
        if self.time_step % 30 == 0:
            print " "
            print "Save network!"
            print "state_batch: %s" %state_batch
            print "action_batch: %s" %action_batch
            print "reward_batch: %s" %reward_batch
            print "Q_value: %s" %self.Q_value
            print Q_value_batch
            self.saver.save(self.session, 'saved_networks/' + 'network' + '-dqn', global_step = self.time_step)

    def guide_action(self,guide):
        if guide == 1:
            out_action = [1,3]
            act_cmd = random.sample(out_action, 1)[0]
        else: 
            in_action = [0,2]
            act_cmd = random.sample(in_action, 1)[0]

        return str(act_cmd)

    def egreedy_action(self,state):
        Q_value = self.Q_value.eval(feed_dict = {
            self.state_input:[state]
            })[0]
        if random.random() <= self.epsilon:
            act_cmd = random.randint(0,self.action_dim - 1)
        else:
            act_cmd = np.argmax(Q_value)


        self.epsilon -= (INITIAL_EPSILON - FINAL_EPSILON) / 10000
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
EPISODE = 100 # Episode limitation
STEP = 20 # Step limitation in an episode
TEST = 10 # The number of experiment test every 100 episode

def main():
    # initialize OpenAI Gym env and dqn agent
    RobotStartUp()
    env = ENV()
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
            print "Episode: %s, Step: %s" %(episode, step)
            
            # ------------------guide--------------------- 
            if guide != 0:
                print "Give a guide:"
                action_guide = raw_input() # a very direct guide which should be changed later as a skin feedback
                                           # which to select a relating direction of action
                if (action_guide == '0') or (action_guide == '1') or (action_guide == '2') or (action_guide == '3'):
                    pass
                else:
                    print "Give a guide again:"
                    action_guide = raw_input()
                action = str(action_guide)
            else:
                action = agent.egreedy_action(state) # e-greedy action for train
            
            action = agent.egreedy_action(state)
            joints = env.getJoint()
            time.sleep(0.2)
            joints = env.getJoint()
            done = env.ActPerfm(action, joints)
            print "Did the robot perform this action?"
            print "0-shoulderin 1-shoulderout 2-elbowin 3-elbowout 9-done correctly"
            done_flag = raw_input()
            if (done_flag != 0) or done_flag!=1 or done_flag != 2 or done_flag != 3 or done_flag != 9:
                print "agian!"
                done_flag = raw_input()
            while done_flag != "9":
                print "0-shoulderin 1-shoulderout 2-elbowin 3-elbowout 9-done correctly"
                joints = env.getJoint()
                time.sleep(0.3)
                joints = env.getJoint()
                env.RepeatPerfm(done_flag, joints)
                done_flag = raw_input()
                if (done_flag != 0) or done_flag!=1 or done_flag != 2 or done_flag != 3 or done_flag != 9:
                    print "agian!"
                    done_flag = raw_input()

            joints = env.getJoint()
            time.sleep(0.3)
            joints = env.getJoint()
            next_state = env.calState(joints)
            '''
            # --------- stop for collecting reward ------------
            print "Skin will collect reward:"
            keyboard_in = raw_input()
            if (keyboard_in == '0'):
                pass
            else:
                pass
            time.sleep(0.2)
            
            reward, guide = env.calReward() # original reward func need guide info as well, abandon!
            #time.sleep(0.2)
            # print "action: %s, state: %s, reward: %s, guide: %s(1: move out | -1: move in), done: %s" %(action,state,reward,guide,done)
            #time.sleep(1)
            # next_state,reward,done,_ = env.step(action) # these three results can be calculate independently
            # Define reward for agent
            # reward_agent = -1 if done else 0.1
            '''
            '''
            state_buffer.append(state)
            action_buffer.append(action)
            next_state_buffer.append(next_state)
            done_buffer.append(done)
            '''
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
            agent.perceive(state,int(action),reward,next_state,done)
            state = next_state
            if done:
                RobotInit() # everytime when a episode is finishing, go back to initial position
                break
        
        
        
        
        
        
        '''
        # Test every 100 episodes
        if (episode % 100 == 0) and (episode != 0):
            print " "
            print "TEST"
            total_reward = 0
            for i in xrange(TEST):
                print "%sth Test:" %i
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
                    print "Skin will collect reward:"
                    keyboard_in = raw_input()
                    if (keyboard_in == '0'):
                        pass
                    else :
                        pass
                    time.sleep(1)
                    reward, guide = env.calReward()
                    time.sleep(1)
                    print "action: %s, state: %s, reward: %s, done: %s" %(action,state,reward,done)
                    total_reward = total_reward + reward
                    print "total_reward: %s" %total_reward
                    if done:
                        RobotInit()
                        break
            ave_reward = total_reward/TEST
            print 'episode: ',episode,'Evaluation Average Reward:',ave_reward
            if ave_reward >= 2000:
                break
            '''

    # save results for uploading
    #env.monitor.start('gym_results/CartPole-v0-experiment-1',force = True)
    '''
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
            #print "Skin will collect reward:"
            #keyboard_in = raw_input()
            #if (keyboard_in == '0'):
                #pass
            #time.sleep(1)
            #reward, guide = env.calReward()
            #time.sleep(1)
            #print "action: %s, state: %s, reward: %s, done: %s" %(action,state,reward,done)
            #total_reward = total_reward + (100 - reward)
            if done:
                RobotInit()
                break
    # env.monitor.close() 
    '''

if __name__ == '__main__':
    rospy.init_node('central_node', anonymous = False)
    main()

