#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from naoqi import ALProxy

nao_ip = "10.0.29.2"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)

def pubState():
    pub = rospy.Publisher('get_state', String, queue_size = 10)
    #pub = rospy.Publisher('get_joint', String, queue_size = 10)
    rospy.init_node('state_pub', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    '''pub = rospy.Publisher('get_joint', Float32MultiArray, queue_size = 1)
    rospy.init_node('joint_pub', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    mat = Float32MultiArray()
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim.append(MultiArrayDimension())
    mat.layout.dim[0].label = "height"
    mat.layout.dim[1].label = "width"
    mat.layout.dim[0].size = 1
    mat.layout.dim[1].size = 1
    mat.layout.dim[0].stride = 1.0*1.0
    mat.layout.dim[1].stride = 1.0
    mat.layout.data_offset = 0
    mat.data = [0]'''

    # get joint value method: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint-api.html#ALMotionProxy::getAngles__AL::ALValueCR.bCR
    # The names of differnt joints: http://doc.aldebaran.com/1-14/naoqi/motion/control-joint.html
    # Also, the name could be "Body", "RArm", "RElbowYaw"
    #names         = "LShoulderPitch"
    #useSensors  = True
    

    while not rospy.is_shutdown():
        theta1 = motion.getAngles("LShoulderPitch", True)
        theta2 = motion.getAngles("RElbowYaw",      True)
        angle = theta1+theta2
        # mat.data[0] = sensorAngles
        # rospy.loginfo(mat.data)
        mat = str(theta1[0])
        mat2 = str(theta2[0])
        rospy.loginfo(mat)
        #rospy.loginfo(mat2)
        # rospy.loginfo(type(mat)
        pub.publish(mat) # later change here publish dierect the state number!!!!!!
        #pub.publish(mat2)
        rate.sleep()

if __name__ == '__main__':
    try:
        pubState()
    except rospy.ROSInterruptException:
        rospy.loginfo("GoForward node terminated.")