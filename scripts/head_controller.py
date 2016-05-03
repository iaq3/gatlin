#!/usr/bin/env python

import rospy, sys, tf
from math import *
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *
from tf.transformations import *
from copy import deepcopy

class Head_Controller:
	def __init__(self):
		# Give the launch a chance to catch up
		rospy.sleep(5)

		rospy.init_node('Head_Controller')
		rospy.loginfo("Launched Head Controller")

		self.REFERENCE_FRAME = "head_base_link"

		self.head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64, queue_size=1)
		self.head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64, queue_size=1)

		self.head_pos_sub = rospy.Subscriber("/head_pos", Vector3, self.head_callback, queue_size=3)

		move_head_service = createService('gatlin/move/head', MoveRobot, self.handle_move_head)
		self.test_head_pose_pub = rospy.Publisher('/test_head_pose', PoseStamped, queue_size=1)

		self.tfl = tf.TransformListener()

		rospy.sleep(5)

		# center the head
		# self.head_set(0.0,0.0)

		# lower the head to look at the floor
		# req = MoveRobotRequest()
		# req.action = "LOOK_DOWN"
		# self.handle_move_head(req)

		# Test LookAt
		req = MoveRobotRequest()
		req.action = "LOOK_AT"
		req.ps.header.frame_id = "base_link"
		# req.ps.header.frame_id = "global_map"
		req.ps.header.stamp = rospy.Time.now()
		req.ps.pose.position = Point(.1, 0, 0)
		req.ps.pose.orientation = Quaternion(0, 0, 0, 1)

		self.handle_move_head(req)

		rospy.spin()

	def handle_move_head(self, req):
		success = True

		if req.action == "ORIENTATION":
			pass

		elif req.action == "LOOK_DOWN" :
			rospy.loginfo("Looking Down")
			self.head_set(0.0,1.17)

		elif req.action == "LOOK_FORWARD" :
			rospy.loginfo("Looking Forward")
			self.head_set(0.0,0.0)

		elif req.action == "LOOK_UP" :
			rospy.loginfo("Looking Up")
			self.head_set(0.0,-1.00)

		elif req.action == "LOOK_AT" :
			rospy.loginfo("Trying to Look At Pose")
			success = self.LookAt(req.ps)

		else :
			success = False
			rospy.logerr("invalid action")

		return MoveRobotResponse(success)

	def unit_vector(self, vector):
		return vector / np.linalg.norm(vector)

	def angle(self, v1, v2):
		v1_u = self.unit_vector(v1)
		v2_u = self.unit_vector(v2)
		cp = np.cross(v1_u,v2_u)
		return copysign(np.arccos(np.dot(v1_u, v2_u)), cp.item(2))

	def head_set(self, pan, tilt):
		# add joint limit checking
		# min_pan = 
		# max_pan = 
		# min_tilt = 
		# max_tilt = 
		# if pan < min_pan or pan > max_pan:
		# 	rospy.logerr("pan is out of range: %f" % pan)
		# 	return False
		# if tilt < min_tilt or tilt > max_tilt:
		# 	rospy.logerr("tilt is out of range: %f" % tilt)
		# 	return False

		rospy.loginfo("pan: %f \t tilt: %f" % (pan, tilt))
		self.head_pan_pub.publish(pan)
		self.head_tilt_pub.publish(tilt)

		return True

	def head_callback(self, data):
		if(data.x == -1.0):
			self.head_set(data.y,data.z)
			return

		headPt = PointStamped()
		headPt.header.frame_id = self.REFERENCE_FRAME
		headPt.header.stamp = rospy.Time.now()
		headPt.point = Point(-data.x,data.y,data.z)

		self.head_set_xyz(headPt)

	def LookAt(self, ps) :
		look_at_ps = self.transform_pose(self.REFERENCE_FRAME, ps)

		self.test_head_pose_pub.publish(look_at_ps)

		headPt = PointStamped()
		headPt.header.frame_id = self.REFERENCE_FRAME
		headPt.header.stamp = rospy.Time.now()
		headPt.point = deepcopy(look_at_ps.pose.position)
		
		return self.head_set_xyz(headPt)

	def head_set_xyz(self, pts):
		x = pts.point.x
		y = pts.point.y
		z = pts.point.z
		rotx = atan2( y, (x**2+z**2)**.5 )
		roty = atan2( x * cos(rotx), z )

		return self.head_set(roty,-rotx)

	# transform the pose stamped to the new frame
	def transform_pose(self, new_frame, pose):
		if pose.header.frame_id == new_frame:
			return pose
		try:
			ps = deepcopy(pose)
			ps.header.stamp = rospy.Time(0)
			self.tfl.waitForTransform(ps.header.frame_id, new_frame, rospy.Time(0), rospy.Duration(4.0))
			new_pose = self.tfl.transformPose(new_frame, ps)
			new_pose.header.stamp = deepcopy(pose.header.stamp)
			return new_pose
		except Exception as e:
			rospy.logerr(e)
			rospy.logerr("no transform")
			return None

if __name__ == "__main__":
	Head_Controller()