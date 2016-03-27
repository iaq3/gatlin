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

		ar_marker_topic = "/ARmarker_points"
		self.tf_sub = rospy.Subscriber("/tf", tfMessage, self.tf_callback, queue_size=1)

		self.objectlistpub = rospy.Publisher("/gatlin/ar_marker_list", ObjectList, queue_size=3)

		self.tfl = tf.TransformListener()
		
		self.CAMERA_FRAME = "camera_rgb_optical_frame"
		self.FIXED_FRAME = "odom"
		self.BASE_FAME = "base_link"

		self.br = tf.TransformBroadcaster()

		self.ar_marker_dict = {}
		self.ar_marker_dict["0"] = "gatlin_arm"
		self.ar_marker_dict["3"] = "baxter"

		rospy.spin()

	def tf_callback(self, tfmsg):
		for trans in tfmsg.transforms:
			if trans.child_frame_id.startswith("ar_marker_"):

				i = trans.child_frame_id.split("ar_marker_",1)[1]

				if i == "255": return

				# TODO: republish for use later or 
				# maybe use the frame as the parent of baxter
				# create static transform from ar_marker_# to baxter base link
				# also filter with an EKF
				if self.ar_marker_dict[i] == "baxter":
					broadcastTransform(i, trans)

				# gatlin_arm calibrate
				if self.ar_marker_dict[i] == "gatlin_arm":
					self.calibrate_arm(trans)

	def broadcastTransform(i, trans):
		# transform to fixed global map frame
		try:
			frame_id = self.ar_marker_dict[i]
		except:
			rospy.logerr("No attached frame for marker %s" % i)
			return

		T = trans.transform.translation
		R = trans.transform.rotation

		self.br.sendTransform(
			(T.x, T.y, T.z),
			(R.x, R.y, R.z, R.w),
			rospy.Time.now(),
			"%s_link" % self.ar_marker_dict[i],
			"global_map"
		)

	def add_object(self):
		# ps = PoseStamped()
		# ps.header.frame_id = self.CAMERA_FRAME
		# ps.header.stamp = rospy.Time.now()
		# ps.pose = deepcopy(bestPose)
		# # rospy.logerr(ps)

		# o = Object()
		# o.id = "%d" % i
		# o.color = hsv_mask.color
		# o.pose = deepcopy(ps)
		# # rospy.logerr(o)
		# self.objectlist.objects.append(o)

	def calibrate_arm(self, trans):
		(trans_act,rot_act) = self.getTransform(self.BASE_FAME,trans.child_frame_id)
		(trans_exp,rot_exp) = self.getTransform(self.BASE_FAME,"expected_"+trans.child_frame_id)
		
		np_trans_act = np.array(trans_act)
		np_trans_exp = np.array(trans_exp)
		trans_error = np_trans_act-np_trans_exp
		rospy.logerr(trans_error)

	def getTransform(self, parent, child):
		# self.tfl.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(4.0))
		(trans,rot) = self.tfl.lookupTransform(parent, child, rospy.Time(0))
		return (trans,rot)

if __name__ == "__main__":
	AR_Vision()