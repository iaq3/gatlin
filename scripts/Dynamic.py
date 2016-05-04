#!/usr/bin/env python
# Names: Zach Vinegar, Isaac Qureshi
import rospy, tf, time
import numpy as np
from geometry_msgs.msg import *
from tf.transformations import *
from copy import deepcopy
from gatlin.msg import *

class DynamicManager:
	def __init__(self, tfl):
		self.tfl = tfl
		self.dynamic_poses = []
		self.ol_subs = []

	def create_dp(self, output_frame):
		dp = DynamicPose(output_frame, self.tfl)
		self.dynamic_poses.append(dp)
		return dp

	def objectListCallback(self, ol):
		# rospy.logerr(ol)
		for obj in ol.objects:
			for dp in self.dynamic_poses:
				if dp.is_obj(obj):
					dp.set_pose(obj.pose)
					# rospy.logerr(dp.ps)
					# rospy.logerr(obj.pose)

	def add_ol_sub(self, ol_topic):
		ol_sub = rospy.Subscriber(ol_topic, ObjectList, self.objectListCallback, queue_size = 1)
		self.ol_subs.append(ol_sub)

	def unregister_all(self):
		for ol_sub in self.ol_subs:
			ol_sub.unregister()
		self.ol_subs = []

class DynamicPose:
	def __init__(self, output_frame, tfl):
		# self.output_frame = "global_map"
		self.output_frame = output_frame
		self.tfl = tfl
		self.ps = PoseStamped()
		self.last_update = 0
		self.color = ""
		self.id = ""

	def get_name(self):
		return "%s_%s" % (self.color, self.id)

	def subscribe_name(self, name):
		color, ID = name.split("_")
		self.color = color
		self.id = ID

	def subscribe(self, color, ID):
		self.color = color
		self.id = ID

	def is_obj(self, obj):
		return self.color == obj.color and self.id == obj.id

	def set_pose(self, ps):
		self.ps = self.transform_pose(self.output_frame, ps)
		self.last_update = time.time()

	# transform the pose stamped to the new frame
	def transform_pose(self, new_frame, pose):
		if pose.header.frame_id == new_frame:
			return pose
		# try:
		ps = deepcopy(pose)
		ps.header.stamp = rospy.Time(0)
		self.tfl.waitForTransform(ps.header.frame_id, new_frame, rospy.Time(0), rospy.Duration(4.0))
		new_pose = self.tfl.transformPose(new_frame, ps)
		new_pose.header.stamp = deepcopy(pose.header.stamp)
		return new_pose
		# except Exception as e:
		# 	rospy.logerr(e)
		# 	rospy.logerr("no transform %s -> %s" % (ps.header.frame_id, new_frame))
		# 	return None