#!/usr/bin/env python

from threading import Thread
import rospy, sys, tf
from math import *
from geometry_msgs.msg import *
from tf.transformations import *
from tf import *
from copy import deepcopy
#from turtle_sim.msg import Velocity
	
class Tf_Transformer(Thread):
	def __init__(self, tfl, child, parent, rate):
		Thread.__init__(self)
		self.setDaemon(True)

		self.tfl = tfl
		self.child = child
		self.parent = parent
		self.pose_pub = rospy.Publisher('%s_in_%s' % (child, parent), Pose, queue_size=1)
		self.rate = rate

		self.child_pose = None

	def run(self):
		self.frames = []
		while not rospy.is_shutdown() and len(self.frames) <= 0:
			self.frames = self.tfl.getFrameStrings()
			self.rate.sleep()

		print self.frames

		while not rospy.is_shutdown():
			self.get_pose()
			
			self.rate.sleep()


	def get_pose(self):
		try:
			ps = PoseStamped()

			ps.pose.orientation = Quaternion(0,0,0,1)
			ps.pose.position = Point(0,0,0)

			# print "self.child, self.parent"
			# print self.child, self.parent

			ps.header.frame_id = self.child
			# ps.header.stamp =  self.tfl.getLatestCommonTime(self.child, self.parent)
			ps.header.stamp =  rospy.Time(0)
			self.tfl.waitForTransform(self.child, self.parent, ps.header.stamp, rospy.Duration(4.0))
			self.child_pose = self.tfl.transformPose(self.parent, ps)

			if self.child_pose != None:
				self.pose_pub.publish(self.child_pose.pose)
				print '%s_in_%s' % (self.child, self.parent)
				print self.child_pose.pose

		except Exception as e:
			print e
			print "no transform"


class base_visual_servo:
	def __init__(self, target):
		rospy.init_node('base_visual_servo')

		self.green_pose = Pose()
		self.execute_servo = False;

		rospy.Subscriber("/base_servo_command", Int32, queue_size = 2)
		rospy.Subscriber("/robot_pose", Pose, queue_size =1)
		rospy.Subscriber("/green_kinect_pose", Pose, queue_size = 1)
		#assume ball is on the ground, simply use the poses
		#robot


		ps = Header()
		ps.frame_id = 'kinect'
		ps.header.stamp =  rospy.Time(0)
		self.tfl.waitForTransform(self.child, self.parent, ps.header.stamp, rospy.Duration(.1))
		self.child_pose = self.tfl.transformPose(self.parent, ps)


		#need green kinect pose in the base frame


if __name__ == "__main__":
	base_visual_servo()