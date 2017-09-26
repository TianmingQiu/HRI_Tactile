#! /user/bin/env python

import rospy
import sys
from naoqi import ALProxy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import time



nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)
touch = 0
left = 0
right = 0



def push_pull(data):
    global left
    global right
    global error
    arm_inside = 0
    for i in range(0, 8):
        arm_inside = arm_inside + data.data[i]

    arm_outside = 0
    for i in range(8,16):
        arm_outside = arm_outside + data.data[i]
    error = arm_inside - arm_outside
    if arm_inside > arm_outside:
         left = 1
    else:
        right = 1


def left_arm(data):
    global touch
    touch = any(j >= 0.1 for j in data.data)

def joint_CB(data):
    #global joint_value
    #joint_value = str(data.data)
    #print "hi"
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def getState():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, joint_CB)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    #print joint_value



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

'''def joint_pub():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('joint_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # get joint value method: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint-api.html#ALMotionProxy::getAngles__AL::ALValueCR.bCR
    # The names of differnt joints: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint.html
    # Also, the name could be "Body", "RArm"
    names         = "LShoulderPitch"
    useSensors  = True
    

    while not rospy.is_shutdown():
        sensorAngles = motion.getAngles(names, useSensors)
        hello_str = str(sensorAngles)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()'''




if __name__ == '__main__':
    motion.wakeUp()
    posture.goToPosture("StandInit", 0.5)

    rospy.init_node("central_node", anonymous = True)
    #rospy.Subscriber("range_left_arm_upper", Float32MultiArray, left_arm)
    #rospy.Subscriber("force_left_arm_upper", Float32MultiArray, push_pull)

    
    motion.setStiffnesses("Body", 1.0)
    motion.moveInit()
    # getState()





    while not rospy.is_shutdown():
        time.sleep(0.1)
        getState()
        
        '''if touch:
            if error > 0.1:
                print "actl"
                time.sleep(0.1)
                rospy.loginfo("\n go left")
                motion.post.moveTo(0, 0.1, 0)'''

    #