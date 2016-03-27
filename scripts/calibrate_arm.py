#!/usr/bin/env python

from threading import Thread
import rospy, sys, tf
from math import *
from geometry_msgs.msg import *
from tf.transformations import *
from tf import *
from copy import deepcopy

class Tf_Calibrate:
	def __init__(self):
		rospy.init_node('Tf_Calibrate')

		self.tfl = TransformListener()

		GRIPPER_FRAME = 'gripper_active_link'
		TOOL_FRAME = 'tool_tip_link'
		BASE_FRAME = 'base_link'
		ARM_BASE_FRAME = 'arm_base_link'
		HEAD_FRAME = 'head_base_link'
		KINECT_FRAME = 'camera_link'
		CAMERA_RGB_OPTICAL_FRAME = 'camera_rgb_optical_frame'

		ACTUAL_FRAME = "ar_marker_0"
		EXPECTED_FRAME = "expected_ar_marker_0"
		
		rospy.spin()

	# def run(self):
	# 	self.frames = []
	# 	while not rospy.is_shutdown() and len(self.frames) <= 0:
	# 		self.frames = self.tfl.getFrameStrings()
	# 		self.rate.sleep()

	# 	print self.frames

	# 	while not rospy.is_shutdown():
	# 		self.get_pose()
			
	# 		self.rate.sleep()

	# def get_pose(self):
	# 	try:
	# 		ps = PoseStamped()

	# 		ps.pose.orientation = Quaternion(0,0,0,1)
	# 		ps.pose.position = Point(0,0,0)

	# 		ps.header.frame_id = self.child
	# 		# ps.header.stamp =  self.tfl.getLatestCommonTime(self.child, self.parent)
	# 		ps.header.stamp =  rospy.Time(0)
	# 		self.tfl.waitForTransform(self.child, self.parent, ps.header.stamp, rospy.Duration(4.0))
	# 		self.child_pose = self.tfl.transformPose(self.parent, ps)

	# 		if self.child_pose != None:
	# 			self.pose_pub.publish(self.child_pose)
	# 			print '%s_in_%s' % (self.child, self.parent)
	# 			print self.child_pose

	# 	except Exception as e:
	# 		print e
	# 		rospy.logerr("no transform %s_in_%s" % (self.child, self.parent))

if __name__ == "__main__":
	Tf_Calibrate()