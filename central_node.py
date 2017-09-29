#!/usr/bin/env python
# Author: Tianming Qiu

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from naoqi import ALProxy
import time

nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)

class ENV():
    def __init__(self):
        self.reward = 0
        self.state = 0
        self.state_dim = 2
        self.action_dim = 4

    def calReward(self):
        rospy.Subscriber("force_hand", Float32MultiArray, self.reward_CB)
        #sleep 
        #rospy.spin()
        return self.reward

    def reward_CB(self, data):
        self.reward = 0
        for i in range(14):
            self.reward = self.reward + data.data[i]
        self.reward = self.reward + 5 * (data.data[0] + data.data[1] + data.data[9] + data.data[11])
        

    def getState(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # node are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        #rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("get_state", String, self.joint_CB)

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        #print joint_value
        return self.state

    def joint_CB(self, data):
        #global joint_value
        #joint_value = str(data.data)
        #print "hi"
        #rospy.loginfo(data.data)
        self.state = data.data

def RobotInit():
    motion.wakeUp()
    posture.goToPosture("Crouch", 1.0)
    motion.waitUntilMoveIsFinished() # should be considered!!!!!!
    names  = ["Body"]
    angles  = [-0.038392066955566406, 0.1349501609802246, 1.1964781284332275, 0.07512402534484863, -1.4926238059997559, -1.3391400575637817, 0.11500811576843262, 0.029999971389770508, -0.25766992568969727, -0.09506607055664062, -0.9694461822509766, 2.086198091506958, -1.168950080871582, 0.07367396354675293, -0.25766992568969727, 0.10128593444824219, -0.9342479705810547, 2.0663399696350098, -1.186300277709961, -0.07205605506896973, -0.309826135635376, 0.24233007431030273, 0.06131792068481445, 0.8544800281524658, 1.5983860492706299, 0.17799997329711914]
    fractionMaxSpeed  = 0.2
    time.sleep(1)
    motion.setAngles(names, angles, fractionMaxSpeed)

    time.sleep(1)
    motion.openHand('RHand')
    time.sleep(2)
    motion.closeHand('RHand')
    time.sleep(5)

if __name__ == '__main__':
    rospy.init_node('central_node', anonymous = False)
    RobotInit()
    env = ENV()

    try:
        #rospy.spin()
        while not rospy.is_shutdown():
            time.sleep(0.1)
            #rew = env.calReward()
            state = env.getState()
            state = float(state)
            #print rew
            print state
            #print type(state)
            

    except rospy.ROSInterruptException:
        pass