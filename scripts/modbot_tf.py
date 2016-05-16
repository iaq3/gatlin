#!/usr/bin/env python

from config import *
import numpy as np
import rospy
from geometry_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *
import tf
from tf.msg import *
from tf.transformations import *

class Modbot:
	def __init__(self):
		rospy.init_node('modbot_tf')

        rospy.Subscriber("/modbot_pose", PoseStamped, self.modbot_pose_cb, queue_size=1)

		self.tfb = tf.TransformBroadcaster()
		
		rospy.sleep(1)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.broadcast_transform()
			rate.sleep()

		rospy.spin()

	def modbot_pose_cb(self, ps):
		pass

	def broadcast_transform(self):
		self.tfb.sendTransformMessage(f.ts)
		rospy.loginfo("Published Modbot Transform")
		
if __name__ == '__main__':
	fm = FM()
