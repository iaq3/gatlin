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

		rospy.spin()

	def tf_callback(self, tfmsg):
		for trans in tfmsg.transforms:
			if trans.child_frame_id.startswith("ar_marker_"):
				i = trans.child_frame_id.split("ar_marker_",1)[1]
				rospy.logerr(i)
				# rospy.logerr(trans.header.frame_id)
				rospy.logerr(trans.child_frame_id)
				rospy.logerr(trans.transform)

				# TODO: republish for use later or 
				# maybe use the frame as the parent of baxter
				# create static transform from ar_marker_# to baxter base link

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

if __name__ == "__main__":
	AR_Vision()