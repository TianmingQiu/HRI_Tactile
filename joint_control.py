#! /user/bin/env python
import rospy
from naoqi import ALProxy
from std_msgs.msg import Float32MultiArray
import time

nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)

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
    posture.goToPosture("Crouch", 1.0)
    motion.waitUntilMoveIsFinished()
    rospy.init_node("joint_control", anonymous=True)
    names  = ["Body"]
    angles  = [-0.02611994743347168, 0.11194014549255371, 1.221022129058838, 0.06745409965515137, -1.5187020301818848, -1.3498780727386475, 0.1487560272216797, 0.02799999713897705, -0.25766992568969727, -0.09506607055664062, -0.9694461822509766, 2.086198091506958, -1.168950080871582, 0.07367396354675293, -0.25766992568969727, 0.10128593444824219, -0.9342479705810547, 2.0694079399108887, -1.186300277709961, -0.07205605506896973, -0.08893013000488281, -0.04606199264526367, 0.2239220142364502, 1.02168607711792, 1.1688660383224487, 0.0307999849319458]
    fractionMaxSpeed  = 0.2
    time.sleep(1)
    motion.setAngles(names, angles, fractionMaxSpeed)

    time.sleep(1)
    motion.openHand('RHand')
    time.sleep(2)
    motion.closeHand('RHand')
    '''flag = 10
    angle = 0.2
    time.sleep(10)
    while flag != 0:
        angle = angle + 0.1

        motion.setAngles('RElbowRoll', [angle], fractionMaxSpeed)
        flag = flag - 1
        time.sleep(1.5)'''


    #motion.setStiffnesses("Body", 1.0)
    #motion.moveInit()
    #motion.setStiffnesses("RArm", 1.0)

    '''while not rospy.is_shutdown():
        time.sleep(0.1)
        if touch:
            if error > 0.1:
                print "actl"
                time.sleep(0.1)
                rospy.loginfo("\n go left")
                motion.post.moveTo(0, 0.1, 0)'''