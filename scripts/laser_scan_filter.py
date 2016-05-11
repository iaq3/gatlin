#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from copy import deepcopy
from sensor_msgs.msg import *
	
class LaserScanFilter:
	def __init__(self):
		rospy.init_node('Laser_Scan_Filter')

		# subscribe to the laser scan
		# subscribe to the point cloud

		# get the xyz of all points in the laser scan in the base_link frame
		# if most of the points are close to the ground, filter out the laser scan

		# if the robot is pointed toward another mobile robot filter out the scan

		# topic = "/cmd_vel_mux/input/teleop"
		# new_topic = "/cmd_vel"
		# self.new_topic_pub = rospy.Publisher(new_topic, Twist, queue_size=1)
		# self.topic_sub = rospy.Subscriber(topic, Twist, self.topic_callback, queue_size=1)


		# subscribe to the depth camera info
		rospy.Subscriber("/camera/depth/camera_info", CameraInfo, self.depth_camera_info_cb, queue_size=1)

		# republish in new frame
		self.camera_info_pub = rospy.Publisher("/camera/plannar_rgb/camera_info", CameraInfo, queue_size=1)

		rospy.spin()

	# def topic_callback(self, data):
	# 	self.new_topic_pub.publish(data)
	# 	rospy.loginfo(data)

	def depth_camera_info_cb(self, data):
		data.header.frame_id = "plannar_optical_frame"
		self.camera_info_pub.publish(data)


if __name__ == "__main__":
	LaserScanFilter()