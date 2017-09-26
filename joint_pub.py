#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from naoqi import ALProxy

nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)

def joint_pub():
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
        rate.sleep()

if __name__ == '__main__':
    try:
        joint_pub()
    except rospy.ROSInterruptException:
        pass
