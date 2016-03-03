#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from copy import deepcopy
	
class Remapper:
	def __init__(self):
		rospy.init_node('Remapper')

		topic = "/cmd_vel_mux/input/teleop"
		new_topic = "/cmd_vel"
		self.new_topic_pub = rospy.Publisher(new_topic, Twist, queue_size=1)
		self.topic_sub = rospy.Subscriber(topic, Twist, self.topic_callback, queue_size=1)

		rospy.spin()

	def topic_callback(self, data):
		self.new_topic_pub.publish(data)
		rospy.loginfo(data)

if __name__ == "__main__":
	Remapper()