#! /user/bin/env python

import rospy
from naoqi import ALProxy
from std_msgs.msg import Float32MultiArray
import time

nao_ip = "10.0.29.2"
#nao_ip = "192.168.0.100"
# nao_ip = "127.0.0.1"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)

Chest = 0
Back = 0


def front(data):
    global Chest
    global Back

    Chest = any(j >= 0.1 for j in data.data)


def back(data):
    global Chest
    global Back

    Back = any(j >= 0.1 for j in data.data)


def nao_control():
    if Chest and Back:
        # tts.say("I confuse where to go")
        # rospy.loginfo("\n Stand")
        # posture.goToPosture("StandInit",1.0)
        return

    elif Chest and not Back:
        # tts.say("go right")
        rospy.loginfo("\n Go backward")
        while True:
            motion.post.moveTo(-0.05, 0, 0)
            if not Chest and not Back:
                motion.waitUntilMoveIsFinished()
                break
            elif Chest and Back:
                motion.killMove()
                break
        # motion.waitUntilMoveIsFinished()

    elif Back and not Chest:
        # tts.say("go left")
        rospy.loginfo("\n Go forward")
        while True:
            motion.post.moveTo(0.05, 0, 0)
            if not Chest and not Back:
                motion.waitUntilMoveIsFinished()
                break
            elif Chest and Back:
                motion.killMove()
                break
        # motion.waitUntilMoveIsFinished()


if __name__ == '__main__':
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.5)

    rospy.init_node("front_back", anonymous=True)

    # Chest:
    rospy.Subscriber("range_front", Float32MultiArray, front)
    # Back:
    rospy.Subscriber("range_back", Float32MultiArray, back)

    motion.setStiffnesses("Body", 1.0)
    motion.moveInit()

    while not rospy.is_shutdown():
        time.sleep(0.1)
        print Chest, Back
        if Chest or Back:
            nao_control()
