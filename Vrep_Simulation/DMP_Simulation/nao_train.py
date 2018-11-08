# -*- coding: utf-8 -*-


import vrep,sys
from naoqi import ALProxy
from manage_joints import get_first_handles,JointControl,JointControlFull
import time
from ENV import ENV
import numpy as np
import pandas as pd
import numpy as np


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


data = pd.read_csv('train_data/theta_train_15.csv')
df=np.transpose(data.values)


a = df[0]
b = df[1]

time.sleep(3)

print "start!!"



for i in range(len(a)):
    
    nao.pose(a[i],b[i])
    #time.sleep(0.3)
    print "act"
