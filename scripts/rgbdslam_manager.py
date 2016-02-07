#!/usr/bin/env python

import rospy, sys, tf
import roslaunch

from math import *
import numpy as np

from copy import deepcopy

from std_msgs.msg import String, Float64, Header, Int32

launchFlag = False
killFlag = False

def RgbdslamManager():
	global rgbdslam_process
	global octomap_server_process
	global launchFlag
	global killFlag
	# global arm_target_marker_pub

	print "Launched Rgbdslam Manager"

	rospy.init_node('RgbdslamManager')

	launched = False	
	killed = True	

	rospy.Subscriber("/gatlin_cmd", Int32, cmdCb, queue_size=1)

	launcher = roslaunch.scriptapi.ROSLaunch()
	launcher.start()

	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		if(launchFlag and not launched):

			package = 'rgbdslam'
			executable = 'rgbdslam'
			rgbdslam_node = roslaunch.core.Node(package, executable)
			rgbdslam_process = launcher.launch(rgbdslam_node)	

			rospy.sleep(3)

			set_param()			
			
			package = 'gatlin_octomap'
			executable = 'octomap_server'
			octomap_server_node = roslaunch.core.Node(package, executable)
			octomap_server_process = launcher.launch(octomap_server_node)	
			
			print "################# Launched rgbdslam #################"

			launched = True
			killed = False
			launchFlag = False

		elif(killFlag and not killed):
			
			rgbdslam_process.stop()
			octomap_server_process.stop()
			print "################# Killed rgbdslam #################"

			killed = True
			launched = False
			killFlag = False

		rate.sleep()

def cmdCb(msg):
	# print msg.data

	if(msg.data == 10):
		launch_rgbdslam()
	elif(msg.data == 11):
		kill_rgbdslam()

def launch_rgbdslam():
	global launchFlag
	launchFlag = True

def kill_rgbdslam():
	global killFlag
	killFlag = True

def set_param():
	# /rgbdslam/config/backend_solver
	# /rgbdslam/config/base_frame_name
	# /rgbdslam/config/cloud_creation_skip_step
	# /rgbdslam/config/data_skip_step
	# /rgbdslam/config/feature_detector_type
	# /rgbdslam/config/feature_extractor_type
	# /rgbdslam/config/fixed_frame_name
	# /rgbdslam/config/matcher_type
	# /rgbdslam/config/max_dist_for_inliers
	# /rgbdslam/config/max_keypoints
	# /rgbdslam/config/maximum_depth
	# /rgbdslam/config/min_sampled_candidates
	# /rgbdslam/config/minimum_depth
	# /rgbdslam/config/nn_distance_ratio
	# /rgbdslam/config/octomap_online_creation
	# /rgbdslam/config/octomap_resolution
	# /rgbdslam/config/optimizer_skip_step
	# /rgbdslam/config/ransac_iterations
	# /rgbdslam/config/store_pointclouds
	# /rgbdslam/config/topic_image_depth
	# /rgbdslam/config/topic_image_mono
	# /rgbdslam/config/visualization_skip_step
	# print rospy.get_param_names()

	rospy.set_param('/rgbdslam/config/fixed_frame_name', '/map')
	rospy.set_param('/rgbdslam/config/base_frame_name', '/odom')
	rospy.set_param('/rgbdslam/config/store_pointclouds', True)
	rospy.set_param('/rgbdslam/config/octomap_online_creation', True)
	rospy.set_param('/rgbdslam/config/topic_image_mono', '/camera/rgb/image_rect_color') 
	rospy.set_param('/rgbdslam/config/topic_image_depth', '/camera/depth_registered/hw_registered/image_rect_raw')
	rospy.set_param('/rgbdslam/config/feature_detector_type', 'SURF')
	rospy.set_param('/rgbdslam/config/feature_extractor_type', 'SURF')
	rospy.set_param('/rgbdslam/config/matcher_type', 'FLANN')
	rospy.set_param('/rgbdslam/config/max_keypoints', 300)
	rospy.set_param('/rgbdslam/config/min_sampled_candidates', 5)
	rospy.set_param('/rgbdslam/config/nn_distance_ratio', 0.8)
	rospy.set_param('/rgbdslam/config/max_dist_for_inliers', 2.0)
	rospy.set_param('/rgbdslam/config/ransac_iterations', 100)
	rospy.set_param('/rgbdslam/config/data_skip_step', 1)
	rospy.set_param('/rgbdslam/config/optimizer_skip_step', 5)
	rospy.set_param('/rgbdslam/config/backend_solver', 'pcg')
	rospy.set_param('/rgbdslam/config/cloud_creation_skip_step', 8)
	rospy.set_param('/rgbdslam/config/visualization_skip_step', 2)
	rospy.set_param('/rgbdslam/config/octomap_resolution', 0.05)
	rospy.set_param('/rgbdslam/config/maximum_depth', 3.5)
	rospy.set_param('/rgbdslam/config/minimum_depth', 0.8)

	# print rospy.get_param('/rgbdslam/config/topic_image_mono')


if __name__ == "__main__":
	RgbdslamManager()


