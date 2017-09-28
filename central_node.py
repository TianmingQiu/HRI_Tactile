#!/usr/bin/env python
# license removed for brevity

# https://answers.ros.org/question/248413/return-data-from-a-callback-function-for-use-in-a-different-function/

import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
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
        

if __name__ == '__main__':
    rospy.init_node('central_node', anonymous = False)
    # rospy.Subscriber("force_left_arm_upper", Float32MultiArray, reward_CB)
    env = ENV()
    try:
        #rospy.spin()
        while not rospy.is_shutdown():
            time.sleep(0.1)
            rew = env.calReward()
            state = env.getState()
            state = float(state)
            print rew
            print state
            print type(state)
            

    except rospy.ROSInterruptException:
        pass