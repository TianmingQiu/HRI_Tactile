#! /user/bin/env python

import rospy
import tf
from geometry_msgs.msg import Point
from time import sleep
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker


def wall():
	pub = rospy.Publisher("visualization_marker", MarkerArray, queue_size = 100)
	marker_array = MarkerArray()

	while not rospy.is_shutdown():
		try:
			marker_1 = Marker()
			# Reference frame:
			marker_1.header.frame_id = "odom"
			marker_1.type = marker_1.CUBE
			marker_1.action = marker_1.ADD
			marker_1.id = 0
			marker_1.scale.x = 3.0
			marker_1.scale.y = 0.05
			marker_1.scale.z = 0.4
			marker_1.color.a = 1.0
			marker_1.color.r = 0.5
			marker_1.color.g = 0.5
			marker_1.color.b = 0.5
			# Position of the wall:
			marker_1.pose.position.x = 0.0
			marker_1.pose.position.y = -1.5
			marker_1.pose.position.z = 0.2
			# Direction of the wall (parallel to the X axis of /odom):
			marker_1.pose.orientation.x = 0.0
			marker_1.pose.orientation.y = 0.0
			marker_1.pose.orientation.z = 0.0
			marker_1.pose.orientation.w = 1.0
			marker_array.markers.append(marker_1)

			marker_2 = Marker()
			# Reference frame:
			marker_2.header.frame_id = "odom"
			marker_2.type = marker_2.CUBE
			marker_2.action = marker_2.ADD
			marker_2.id = 1
			marker_2.scale.x = 0.05
			marker_2.scale.y = 3.0
			marker_2.scale.z = 0.4
			marker_2.color.a = 1.0
			marker_2.color.r = 0.5
			marker_2.color.g = 0.5
			marker_2.color.b = 0.5
			# Position of the wall:
			marker_2.pose.position.x = -1.5
			marker_2.pose.position.y = 0.0
			marker_2.pose.position.z = 0.2
			# Direction of the wall (parallel to the Y axis of /odom):
			marker_2.pose.orientation.x = 0.0
			marker_2.pose.orientation.y = 0.0
			marker_2.pose.orientation.z = 0.0
			marker_2.pose.orientation.w = 1.0
			marker_array.markers.append(marker_2)

			pub.publish(marker_array)

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

if __name__ == '__main__':
	try:
		rospy.init_node("wall", anonymous=True)
		wall()
	except rospy.ROSInterruptException:
		pass
