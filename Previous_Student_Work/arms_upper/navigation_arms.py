#! /user/bin/env python

import rospy
from naoqi import ALProxy
from std_msgs.msg import Float32MultiArray
import time


#nao_ip = "127.0.0.1"
#nao_ip = "192.168.0.100"
nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)
left = 0
right = 0


def left_arm(data):
    global left
    left = any(j >= 0.1 for j in data.data)


def right_arm(data):
    global right
    right = any(j >= 0.1 for j in data.data)
    # print left, right


def nao_control():
    if left and right:
        # tts.say("I confuse where to go")
        # rospy.loginfo("\n Stand")
        # posture.goToPosture("StandInit",1.0)
        return
    elif left and not right:
        # tts.say("go right")
        rospy.loginfo("\n go right")
        while True:
            motion.post.moveTo(0, -0.05, 0)
            if not left and not right:
                motion.waitUntilMoveIsFinished()
                break
            elif left and right:
                motion.killMove()
                break
        # motion.waitUntilMoveIsFinished()
    elif not left and right:
        # tts.say("go left")
        rospy.loginfo("\n go left")
        while True:
            motion.post.moveTo(0, 0.05, 0)
            if not left and not right:
                motion.waitUntilMoveIsFinished()
                break
            elif left and right:
                motion.killMove()
                break
        # motion.waitUntilMoveIsFinished()


if __name__ == '__main__':
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.5)

    rospy.init_node("two_arms_upper", anonymous=True)
    rospy.Subscriber("range_left_arm_upper", Float32MultiArray, left_arm)
    rospy.Subscriber("range_right_arm_upper", Float32MultiArray, right_arm)

    motion.setStiffnesses("Body", 1.0)
    motion.moveInit()

    while not rospy.is_shutdown():
        time.sleep(0.1)
        print left, right
        if left or right:
            nao_control()


