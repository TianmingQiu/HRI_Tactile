# -*- coding: utf-8 -*-
"""
@author: Pierre Jacquot
"""
#For more informations please check : http://www.coppeliarobotics.com/helpFiles/en/apiFunctions.htm
import vrep,sys
from naoqi import ALProxy
from manage_joints import get_first_handles,JointControl,JointControlFull
import time
from ENV import ENV
from DQN import DQN


print '================ Program Sarted ================'

vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.2',19999,True,True,5000,5)
if clientID!=-1:
    print 'Connected to remote API server'

else:
    print 'Connection non successful'
    sys.exit('Could not connect')


print "================ Choregraphe's Initialization ================"
print 'Enter your NAO IP'
naoIP = raw_input()
#naoIP = map(str,naoIP.split())
print 'Enter your NAO port'
naoPort = raw_input()
naoPort=int(naoPort)
#naoPort = map(int,naoPort.split())

motionProxy = ALProxy("ALMotion",naoIP, naoPort)
postureProxy = ALProxy("ALRobotPosture", naoIP, naoPort)

#Go to the posture StandInitZero
#posture = 'StandZero'
#print 'Posture Initialization : ' + posture
motionProxy.stiffnessInterpolation('Body', 1.0, 1.0)
postureProxy.goToPosture("StandZero", 1.0)

Head_Yaw=[];Head_Pitch=[];
L_Hip_Yaw_Pitch=[];L_Hip_Roll=[];L_Hip_Pitch=[];L_Knee_Pitch=[];L_Ankle_Pitch=[];L_Ankle_Roll=[];
R_Hip_Yaw_Pitch=[];R_Hip_Roll=[];R_Hip_Pitch=[];R_Knee_Pitch=[];R_Ankle_Pitch=[];R_Ankle_Roll=[];
L_Shoulder_Pitch=[];L_Shoulder_Roll=[];L_Elbow_Yaw=[];L_Elbow_Roll=[];L_Wrist_Yaw=[]
R_Shoulder_Pitch=[];R_Shoulder_Roll=[];R_Elbow_Yaw=[];R_Elbow_Roll=[];R_Wrist_Yaw=[]
R_H=[];L_H=[];R_Hand=[];L_Hand=[];
Body = [Head_Yaw,Head_Pitch,L_Hip_Yaw_Pitch,L_Hip_Roll,L_Hip_Pitch,L_Knee_Pitch,L_Ankle_Pitch,L_Ankle_Roll,R_Hip_Yaw_Pitch,R_Hip_Roll,R_Hip_Pitch,R_Knee_Pitch,R_Ankle_Pitch,R_Ankle_Roll,L_Shoulder_Pitch,L_Shoulder_Roll,L_Elbow_Yaw,L_Elbow_Roll,L_Wrist_Yaw,R_Shoulder_Pitch,R_Shoulder_Roll,R_Elbow_Yaw,R_Elbow_Roll,R_Wrist_Yaw,L_H,L_Hand,R_H,R_Hand]

get_first_handles(clientID,Body)

commandAngles = motionProxy.getAngles('Body', False)

postureProxy.goToPosture("Sit", 1.0)
time.sleep(1)
JointControlFull(clientID,motionProxy,0,Body)

robot_cfg = (motionProxy, clientID, Body)
nao = ENV(robot_cfg)

nao.robot_init()
time.sleep(3)

nao.robot_init()

time.sleep(3)

print "test"
nao.ShoulderIn()
nao.ShoulderOut()
nao.ElbowOut()
nao.ElbowIn()

# ---------------------------------------------------------
# Hyper Parameters
EPISODE = 100 # Episode limitation
STEP = 20 # Step limitation in an episode
TEST = 10 # The number of experiment test every 100 episode
agent = DQN(nao)
print "Ready to train?[y/N]"
keyboard_in = raw_input()
if keyboard_in == 'y':
    pass

for episode in xrange(EPISODE):
    # initialize task
    joints = nao.getJoint()
    state = nao.calState(joints)
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

        joints = nao.getJoint()
        done = nao.ActPerfm(action, joints)

        joints = nao.getJoint()
        next_state = nao.calState(joints)

        print "Skin will collect reward: good: 9 | bad: 0 "
        keyboard_in = raw_input()
        if keyboard_in == '0':
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
            nao.robot_init()  # everytime when a episode is finishing, go back to initial position
            break

