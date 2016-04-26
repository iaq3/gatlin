#!/usr/bin/env python
import time
from threading import Thread, Lock
import rospy, sys, tf
from math import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from tf import *
from tf.msg import *
from tf.transformations import *
from copy import deepcopy
from gatlin.msg import *
from gatlin.srv import *
from config import *

class AR_Vision :

	def __init__(self):

		self.vision_type = "ar"

		rospy.init_node("%s_vision" % self.vision_type)

		print "initializing %s vision" % self.vision_type

		# ar_marker_topic = "/ARmarker_points"
		self.tf_sub = rospy.Subscriber("/tf", tfMessage, self.tf_callback, queue_size=1)

		# self.TYPE = rospy.get_param("type")
		self.TYPE = "baxter_left"
		self.objectlistpub = rospy.Publisher("/%s/ar_marker_list" % self.TYPE, ObjectList, queue_size=3)
		
		self.test_pose_pub = rospy.Publisher("test_pose", PoseStamped, queue_size=1)

		self.tfl = tf.TransformListener()
		
		# self.RGB_FRAME = rospy.get_param("rgb_frame")
		# self.FIXED_FRAME = rospy.get_param("fixed_frame")
		self.FIXED_FRAME = "base"
		# self.BASE_FAME = rospy.get_param("base_frame")

		self.br = tf.TransformBroadcaster()

		rospy.spin()

	def get_transform(self, parent, child):
		self.tfl.waitForTransform(child, parent, rospy.Time(0), rospy.Duration(4))
		(T,R) = self.tfl.lookupTransform(parent, child, rospy.Time(0))
		tf = Transform()
		tf.rotation = Quaternion(R[0],R[1],R[2],R[3])
		tf.translation = Vector3(T[0],T[1],T[2])
		return tf

	# def transform_to_matrix(self, transform):
	# 	t = deepcopy(transform.translation)
	# 	t = [t.x,t.y,t.z]
	# 	T = translation_matrix(t)
	# 	q = deepcopy(transform.rotation)
	# 	q = [q.x,q.y,q.z,q.w]
	# 	R = quaternion_matrix(q)
	# 	return np.dot(T,R)

	# def transform_from_matrix(self, matrix):
	# 	trans = Transform()
	# 	q = quaternion_from_matrix(matrix)
	# 	trans.rotation = Quaternion(q[0],q[1],q[2],q[3])
	# 	t = translation_from_matrix(matrix)
	# 	trans.translation = Vector3(t[0],t[1],t[2])
	# 	return trans

	# def transform_to_pose(self, transform):
	# 	pose = Pose()
	# 	T = deepcopy(transform.translation)
	# 	R = deepcopy(transform.rotation)
	# 	pose.position = Point(T.x, T.y, T.z)
	# 	pose.orientation = Quaternion(R.x, R.y, R.z, R.w)
	# 	return pose

	def tf_callback(self, tfmsg):
		self.objectlist = ObjectList()
		# fixed_to_rgb_matrix = None		

		for trans in tfmsg.transforms:
			if trans.child_frame_id.startswith("ar_marker_"):

				i = trans.child_frame_id.split("ar_marker_",1)[1]
				if i == "255": return

				# if fixed_to_rgb_matrix == None:
				# 	fixed_to_rgb_t = self.get_transform(self.FIXED_FRAME, self.RGB_FRAME)
				# 	fixed_to_rgb_matrix = self.transform_to_matrix(fixed_to_ar_t)

				# rospy.logerr(i)
				# rospy.logerr(trans.header.frame_id)
				# rospy.logerr(trans.child_frame_id)
				# rospy.logerr(trans.transform)

				# TODO: republish for use later or 
				# maybe use the frame as the parent of baxter
				# create static transform from ar_marker_# to baxter base link
				# also filter with an EKF

				# transform to fixed global map frame
				# try:
				# 	frame_id = self.ar_marker_dict[i]
				# except:
				# 	rospy.logerr("No attached frame for marker %s" % i)
				# 	continue

				# T = trans.transform.translation
				# R = trans.transform.rotation

				# self.br.sendTransform(
				# 	(T.x, T.y, T.z),
				# 	(R.x, R.y, R.z, R.w),
				# 	rospy.Time.now(),
				# 	"%s_link" % self.ar_marker_dict[i],
				# 	"global_map"
				# )

				# rgb_to_ar_t = deepcopy(trans.transform)
				# rgb_to_ar_matrix = self.transform_to_matrix(rgb_to_ar_t)

				# fixed_to_ar_matrix = np.dot(fixed_to_rgb_matrix, rgb_to_ar_matrix)
				# fixed_to_ar_t = self.transform_from_matrix(fixed_to_ar_matrix)

				p = Pose()
				p.position = Point()


				fixed_to_ar_t = self.get_transform(self.FIXED_FRAME, trans.child_frame_id)
				# rospy.logerr(fixed_to_ar_t)

				ps = PoseStamped()
				ps.header.frame_id = self.FIXED_FRAME
				ps.header.stamp = rospy.Time.now()
				# ps.pose = transform_to_pose(trans.transform)
				ps.pose = transform_to_pose(fixed_to_ar_t)
				self.test_pose_pub.publish(ps)
				
				o = Object()
				o.id = "%s" % i
				o.color = "ar"
				o.pose = deepcopy(ps)
				# rospy.logerr(o)
				self.objectlist.objects.append(o)
		if len(self.objectlist.objects) > 0:
			self.objectlistpub.publish(self.objectlist)
			# rospy.logerr(self.objectlist)

if __name__ == "__main__":
	AR_Vision()