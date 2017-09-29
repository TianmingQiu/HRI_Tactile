#! /user/bin/env python
import rospy
from naoqi import ALProxy
from std_msgs.msg import Float32MultiArray
import time

nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)




if __name__ == '__main__':
    names  = 'RArm'
    # If only one parameter is received, this is applied to all joints
    stiffnesses  = 0.0
    motion.setStiffnesses(names, stiffnesses)
