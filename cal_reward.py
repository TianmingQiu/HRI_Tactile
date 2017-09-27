#!/usr/bin/env python
# license removed for brevity

# https://answers.ros.org/question/248413/return-data-from-a-callback-function-for-use-in-a-different-function/

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from naoqi import ALProxy

#nao_ip = "10.0.29.2"
#port = 9559
#motion = ALProxy("ALMotion", nao_ip, port)


class ENV():
    def __init__(self):
        self.reward = 0

    def cal_reward(self):
        self.reward = rospy.Subscriber("force_left_arm_upper", Float32MultiArray, reward_CB)

    def reward_CB(self, data):
        self.reward = 0
        for i in range(14):
            self.reward = self.reward + data.data[i]
        self.reward = self.reward + 5 * (data.data[0] + data.data[1] + data.data[9] + data.data[11])
        return self.reward


if __name__ == '__main__':
    try:
        rospy.init_node('cal_reward', anonymous=True)
        # rospy.Subscriber("force_left_arm_upper", Float32MultiArray, reward_CB)
        env = ENV()
        while True:
            rew = env.cal_reward = ("force_left_arm_upper")
            print env.reward
    except rospy.ROSInterruptException:
        pass