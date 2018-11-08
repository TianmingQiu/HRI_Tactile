#! /user/bin/env python

import rospy
import tf
from geometry_msgs.msg import Point
from time import sleep
from visualization_msgs.msg import Marker

def receive():
	rospy.Subscriber("point_torso", Point, addMarker)
	rospy.spin()

def addMarker(data):
	pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
	while not rospy.is_shutdown():
		try:
			x = data.x
			y = data.y
			z = data.z

			marker = Marker()
			marker.header.frame_id = "torso"
			marker.type = marker.ARROW
			marker.action = marker.ADD
			marker.scale.x = 0.1
			marker.scale.y = 0.01
			marker.scale.z = 0.01
			marker.color.a = 1.0
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0
			marker.pose.position.x = x
			marker.pose.position.y = y
			marker.pose.position.z = z
			marker.pose.orientation.x = x
			marker.pose.orientation.y = y
			marker.pose.orientation.z = z
			marker.pose.orientation.w = 1

			pub.publish(marker)
			sleep(0.7)
			marker.color.a = 0
			pub.publish(marker)
			return

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue




if __name__ == '__main__':
	try:
		rospy.init_node("add_marker", anonymous=True)
		receive()
	except rospy.ROSInterruptException:
		pass
