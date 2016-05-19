#!/usr/bin/env python

from threading import Thread
import rospy, sys, tf
from math import *
from geometry_msgs.msg import *
from tf.transformations import *
from tf.msg import *
from tf import *
from copy import deepcopy
	
class Tf_Transformer(Thread):
	def __init__(self, tfl, child, parent, rate):
		Thread.__init__(self)
		self.setDaemon(True)

		self.tfl = tfl
		self.child = child
		self.parent = parent
		self.pose_pub = rospy.Publisher('%s_in_%s' % (child, parent), PoseStamped, queue_size=1)
		self.rate = rate

		self.child_pose = None

	def run(self):
		# self.frames = []
		# while not rospy.is_shutdown() and len(self.frames) <= 0:
		# 	self.frames = self.tfl.getFrameStrings()
		# 	self.rate.sleep()

		# print self.frames

		while not rospy.is_shutdown():
			self.get_pose()
			self.rate.sleep()

	def get_pose(self):
		try:
			ps = PoseStamped()

			ps.pose.orientation = Quaternion(0,0,0,1)
			ps.pose.position = Point(0,0,0)

			ps.header.frame_id = self.child
			# ps.header.stamp =  self.tfl.getLatestCommonTime(self.child, self.parent)
			ps.header.stamp =  rospy.Time(0)
			self.tfl.waitForTransform(self.child, self.parent, ps.header.stamp, rospy.Duration(4.0))
			self.child_pose = self.tfl.transformPose(self.parent, ps)

			if self.child_pose != None:
				self.pose_pub.publish(self.child_pose)
				rospy.loginfo('%s_in_%s' % (self.child, self.parent))
				rospy.loginfo(self.child_pose)

		except:
			rospy.logerr("no transform %s_in_%s" % (self.child, self.parent))

class Tf_Listener:
	def __init__(self, topic, tfb):
		self.tfb = tfb
		self.topic = topic
		rospy.Subscriber(topic, tfMessage, self.tfmsg_cb, queue_size=1)

	def send_transform(self, ts):
		T = ts.transform.translation
		R = ts.transform.rotation
		translation = [T.x, T.y, T.z]
		rotation = [R.x, R.y, R.z, R.w]
		time = rospy.Time.now()
		parent = ts.header.frame_id
		child = ts.child_frame_id
		self.tfb.sendTransform(translation, rotation, time, child, parent)

	def tfmsg_cb(self, tfmsg):
		rospy.loginfo("Recieved Tf on %s" % self.topic)
		for ts in tfmsg.transforms:
			self.send_transform(ts)
		# rospy.loginfo("Broadcasted Transforms")


class Tf_Broadcaster(Thread):
	def __init__(self, rate, topic, tfl):
		Thread.__init__(self)
		self.setDaemon(True)

		self.tfl = tfl
		self.parent_child = []
		self.tfmsg = None
		self.topic = topic
		self.tfmsg_pub = rospy.Publisher(topic, tfMessage, queue_size=1)
		self.rate = rate

	def get_ts(self, parent, child):
		self.tfl.waitForTransform(child, parent, rospy.Time(0), rospy.Duration(4))
		(T,R) = self.tfl.lookupTransform(parent, child, rospy.Time(0))
		tfs = TransformStamped()
		tfs.header.stamp = rospy.Time.now()
		tfs.header.frame_id = parent
		tfs.child_frame_id = child
		tfs.transform.rotation = Quaternion(R[0],R[1],R[2],R[3])
		tfs.transform.translation = Vector3(T[0],T[1],T[2])
		return tfs

	def broadcast_transforms(self):
		self.tfmsg = tfMessage()
		for parent, child in self.parent_child:
			ts = self.get_ts(parent, child)
			# rospy.loginfo("Got transform %s -> %s" % (parent, child))
			self.tfmsg.transforms.append(ts)

		if len(self.tfmsg.transforms) > 0:
			rospy.loginfo("Published Transforms to %s" % self.topic)
			self.tfmsg_pub.publish(self.tfmsg)

	def run(self):
		while not rospy.is_shutdown():
			self.broadcast_transforms()
			self.rate.sleep()


class Tf_to_Unity:
	def __init__(self):
		rospy.init_node('Tf_to_Unity')

		self.tfl = TransformListener()
		self.tfb = TransformBroadcaster()

		outgoing_tf = rospy.get_param("~outgoing")
		incoming_tf = rospy.get_param("~incoming")

		if incoming_tf != "":
			self.tf_l = Tf_Listener("/server/tf", self.tfb)

		if outgoing_tf != "":
			self.tf_b = Tf_Broadcaster(rospy.Rate(1), outgoing_tf, self.tfl)
			self.tf_b.parent_child = [
				["global_map", "gatlin"],
				["global_map", "baxter"]
			]
			self.tf_b.start()

		FIXED_FRAME = 'global_map'
		GRIPPER_FRAME = 'gripper_active_link'
		TOOL_FRAME = 'tool_tip_link'
		BASE_FRAME = 'base_link'
		ARM_BASE_FRAME = 'arm_base_link'
		HEAD_FRAME = 'head_base_link'
		KINECT_FRAME = 'camera_link'
		CAMERA_RGB_OPTICAL_FRAME = 'camera_rgb_optical_frame'

		transform_pubs = []

		kinect_in_base = Tf_Transformer(self.tfl, KINECT_FRAME, BASE_FRAME, rospy.Rate(5)) # use different rates for less msg collisions?
		transform_pubs.append(kinect_in_base)

		gripper_in_base = Tf_Transformer(self.tfl, TOOL_FRAME, BASE_FRAME, rospy.Rate(4))
		transform_pubs.append(gripper_in_base)

		# base_in_fixed = Tf_Transformer(self.tfl, BASE_FRAME, FIXED_FRAME, rospy.Rate(10))
		# transform_pubs.append(gripper_in_base)

		# arm_base_in_base = Tf_Transformer(self.tfl, ARM_BASE_FRAME, BASE_FRAME, rospy.Rate(1))
		# transform_pubs.append(arm_base_in_base)

		for tp in transform_pubs:
			tp.start()

		rospy.spin()

if __name__ == "__main__":
	Tf_to_Unity()