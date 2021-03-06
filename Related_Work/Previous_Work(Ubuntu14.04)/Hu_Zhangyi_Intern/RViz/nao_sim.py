#! /user/bin/env python

import rospy
import tf
from naoqi import ALProxy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
import numpy as np
import math

nao_ip = "127.0.0.1"
port = 9559
motion = ALProxy("ALMotion", nao_ip, port)
posture = ALProxy("ALRobotPosture", nao_ip, port)

def manipulate(data):
	global motion
	rospy.loginfo("The point being clicked is \n%s" % data.point)
	pub_point = rospy.Publisher("point_torso", Point, queue_size=1)
	listener = tf.TransformListener()
	while not rospy.is_shutdown():
		try:
			# Intended to transform the coordinate in /odom to /base_link (same as /torso)
			(trans, rot) = listener.lookupTransform('base_link', 'odom', rospy.Time(0))

			# Four elements of quaternion 'rot':
			x = rot[0]
			y = rot[1]
			z = rot[2]
			w = rot[3]
			print trans
			print rot

			# Translation from /odom to /base_link:
			point_odom = np.array([data.point.x,data.point.y,data.point.z])
			point_trans = point_odom + trans
			#pub_trans = Vector3(point_trans[0], point_trans[1], point_trans[2])
			#pub_point.publish(pub_trans)

			# Conduct the rotation with p' = q p q_inverse:
			point_base = rotation(x,y,z,w,point_trans)
			#point_base = rotation_45(w, point_trans)
			print point_base

			# The value of force is taken as a random number between 0 and 1:
			force = np.random.random_sample()
			rospy.loginfo("The value of force is %s" % force)

			# Different velocities and reaction time are chosen depending on the level of force:
			if force < 0.5:
				x_dir = 0.03
				y_dir = 0.03
				angular = 0.0
			else:
				x_dir = 0.2
				y_dir = 0.2
				angular = math.pi/4 # 45 degrees

			# x in /base_link: point_base[0], y in /base_link: point_base[1], z in /base_link: point_base[2]
			if point_base[1] > 0.1:
				rospy.loginfo("Hit on the left arm")
				motion.post.moveTo(0, -y_dir, 0)
				motion.waitUntilMoveIsFinished()
				return
			elif point_base[1] < -0.1:
				rospy.loginfo("Hit on the right arm")
				motion.post.moveTo(0, y_dir, 0)
				motion.waitUntilMoveIsFinished()
				return
			elif point_base[0] > 0 and 0 < point_base[1] < 0.7 and 0 < point_base[2] < 0.12:
				rospy.loginfo("Hit on the left chest")
				motion.post.moveTo(-x_dir, -y_dir, 0)
				motion.waitUntilMoveIsFinished()
				return
			elif point_base[0] > 0 and -0.7 < point_base[1] < 0  and 0 < point_base[2] < 0.12:
				rospy.loginfo("Hit on the right chest")
				motion.post.moveTo(-x_dir, y_dir, 0)
				motion.waitUntilMoveIsFinished()
				return
			elif point_base[0] < -0.0230 and -0.760 < point_base[1] < 0.760 and -0.0790 < point_base[2] < 0.1297:
				rospy.loginfo("Hit on the back")
				motion.post.moveTo(x_dir, 0, 0)
				motion.waitUntilMoveIsFinished()
				return
			else:
				rospy.loginfo("No sensors in this area")
				return

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

def rotation(x,y,z,w,point):
	q_v = np.array([x,y,z])
	q_w = w
	q_inverse_v = np.array([-x,-y,-z])
	q_inverse_w = w
	p_v = point
	p_w = 0
	# Step 1: q*p
	q_p_cross_v = np.cross(q_v,p_v) + q_w*p_v + p_w*q_v
	q_p_cross_w = q_w*p_w - np.dot(q_v,p_v)
	# Step 2: q*p*q_inverse
	p_new_v = np.cross(q_p_cross_v,q_inverse_v) + q_p_cross_w*q_inverse_v + q_inverse_w*q_p_cross_v
	return p_new_v

def rotation_45(w,point):
	theta = 2*math.acos(w)
	print theta
	cos_theta = math.cos(theta)
	sin_theta = math.sin(theta)
	point_new = np.array([0.0,0.0,0.0])
	point_new[0] = cos_theta*point[0] + sin_theta*point[1]
	point_new[1] = cos_theta*point[1] - sin_theta*point[0]
	point_new[2] = point[2]
	return point_new

def receive():
	rospy.Subscriber("clicked_point", PointStamped, manipulate)
	rospy.spin()

def stop():
	#rospy.init_node("action_pub", anonymous = True)
	pub_cv = rospy.Publisher("cmd_vel", Twist, queue_size = 1)
	motion = ALProxy("ALMotion", "127.0.0.1", 9559)
	motion.setStiffnesses("Body", 1.0)
	motion.moveInit()
	while not rospy.is_shutdown():
		pub_cv.publish(Twist(Vector3(0.0,0.0,0.0),Vector3(0.0,0.0,0.0)))

if __name__ == '__main__':
	try:
		motion.wakeUp()
		posture.goToPosture("StandInit", 0.5)
		rospy.init_node("action_pub", anonymous = True)
		motion.setStiffnesses("Body", 1.0)
		motion.moveInit()
		receive()
	except rospy.ROSInterruptException:
		stop()
		#pass
