import sys
import rospy
import time
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
import numpy as np
import math
from naoqi import ALProxy

if __name__ == '__main__':
	nao_ip="169.254.80.21"
	#nao_ip="192.168.0.1"
	port=9559
	motion = ALProxy("ALMotion",nao_ip,port)
	motion.wakeUp()
	posture = ALProxy("ALRobotPosture",nao_ip,port)
	posture.goToPosture("Stand",1.0)
	time.sleep(2)
