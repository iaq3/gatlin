#!/usr/bin/env python

import rospy, sys, tf
from math import *
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import *
from copy import deepcopy

class Head_Controller:
	def __init__(self):
		# Give the launch a chance to catch up
		rospy.sleep(5)

		rospy.init_node('Head_Controller')
		rospy.loginfo("Launched Head Controller")

		self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64)
		self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64)

		self.head_pos_sub = rospy.Subscriber("/head_pos", Vector3, self.head_callback, queue_size=3)

		rospy.sleep(5)

		# center the head
		# self.head_set(0.0,0.0)

		# lower the head to look at the floor
		self.lowerHeadRad = 1.17
		self.head_set(0.0,self.lowerHeadRad)

		rospy.spin()

	def unit_vector(self, vector):
		return vector / np.linalg.norm(vector)

	def angle(self, v1, v2):
		v1_u = self.unit_vector(v1)
		v2_u = self.unit_vector(v2)
		cp = np.cross(v1_u,v2_u)
		return copysign(np.arccos(np.dot(v1_u, v2_u)), cp.item(2))

	def head_set_xyz(self, pt):
		x = pt.point.x
		y = pt.point.y
		z = pt.point.z
		rotx = atan2( y, (x**2+z**2)**.5 )
		roty = atan2( x * cos(rotx), z )

		self.head_set(roty,-rotx)

	def head_set(self, pan, tilt):
		rospy.loginfo("pan: %f \t tilt: %f" % (pan, tilt))
		self.head_pan_pub.publish(pan)
		self.head_tilt_pub.publish(tilt)

	def head_callback(self, data):
		if(data.x == -1.0):
			self.head_set(data.y,data.z)
			return

		headPt = PointStamped()
		headPt.header.frame_id = "/head_base_link"
		headPt.header.stamp = rospy.Time.now()
		headPt.point = Point(-data.x,data.y,data.z)

		self.head_set_xyz(headPt)

if __name__ == "__main__":
	Head_Controller()