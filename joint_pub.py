#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from naoqi import ALProxy
import roslib; roslib.load_manifest('numpy_tutorial')

from rospy_tutorials.msg import Floats

#nao_ip = "10.0.29.2"
nao_ip = "127.0.0.1"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)

def pubState():
    pub = rospy.Publisher('floats', Floats, queue_size = 10)
    rospy.init_node('joint_pub', anonymous=False)
    rate = rospy.Rate(10) # 10hz

    # get joint value method: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint-api.html#ALMotionProxy::getAngles__AL::ALValueCR.bCR
    # The names of differnt joints: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint.html
    # Also, the name could be "Body", "RArm", "RElbowYaw"

    while not rospy.is_shutdown():
        theta1 = motion.getAngles("RShoulderRoll", True)
        theta2 = motion.getAngles("RElbowRoll", True)
        angle = theta1+theta2
        rospy.loginfo(angle)
        pub.publish(angle) # type(angle) = list
        rate.sleep()

if __name__ == '__main__':
    try:
        pubState()
    except rospy.ROSInterruptException:
        rospy.loginfo("GoForward node terminated.")