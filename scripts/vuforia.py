#!/usr/bin/env python

import rospy, sys, tf
import moveit_commander

from math import *
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion, Point, PointStamped
from std_msgs.msg import String, Float64, Header
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
from tf import TransformListener

from copy import deepcopy

REFERENCE_FRAME = '/base_link'

waypoints = []
done = True
head_pan_pub = rospy.Publisher('/head_pan_joint/command', Float64)
head_tilt_pub = rospy.Publisher('/head_tilt_joint/command', Float64)
marker_pub = rospy.Publisher('/visualization_marker', Marker)
arm_target_marker_pub = rospy.Publisher('/arm_target_marker', Marker)
lowerHeadRad = 1.17

def unit_vector(vector):
	return vector / np.linalg.norm(vector)

def angle(v1, v2):
	v1_u = unit_vector(v1)
	v2_u = unit_vector(v2)
	cp = np.cross(v1_u,v2_u)
	return copysign(np.arccos(np.dot(v1_u, v2_u)), cp.item(2))

def Server():
	global marker_pub
	global arm_target_marker_pub

	global gripper_pub
	global head_pan_pub
	global head_tilt_pub
	global orientation_pub
	global tf
	global lowerHeadRad
	# global done
	# global waypoints

	# Give the launch a chance to catch up
	rospy.sleep(5)
	print "Launched Vuforia Server"

	rospy.init_node('Vuforia_Server')
	# roscpp_initialize(sys.argv)
	# rospy.init_node('Vuforia_Server')
	# waypoints = []

	tf = TransformListener()

	rospy.Subscriber("/target_pos", Vector3, pos_callback, queue_size=3)
	rospy.Subscriber("/head_pos", Vector3, head_callback, queue_size=3)
	# rospy.Subscriber("/target_go", String, go_callback, queue_size=1)


	# init a gripper publisher because movegroup won't work
	gripper_pub = rospy.Publisher('/gripper_joint/command', Float64)


	orientation_pub = rospy.Publisher('/target_orientation', Vector3)

	# center the head
	head_set(0.0,0.0)

	# lower the head to look at the floor
	# head_set(0.0,lowerHeadRad)

	rospy.spin()	

def head_set_xyz(pt):
	# pt.header.stamp -= rospy.Duration(.22)
	# print pt.header.stamp 
	# tf.waitForTransform("/head_base_link", "/camera_rgb_optical_frame", rospy.Time(), rospy.Duration(4.0))
	# kinectPt = tf.transformPoint("/camera_rgb_optical_frame", pt)

	# pt.point.x += -0.012
	# pt.point.y += 0.142
	# pt.point.z += 0.000
	
	# ref = np.array([0,1,0])
	# v1 = np.array([pt.point.x,pt.point.z,0])

	# pan = -angle(ref, v1)

	# xz = np.array([pt.point.x,pt.point.z])	
	# v2 = np.array([np.linalg.norm(xz),pt.point.y,0])

	# tilt = angle(ref, v2)

	x = pt.point.x
	y = pt.point.y
	z = pt.point.z
	rotx = atan2( y, (x**2+z**2)**.5 )
	roty = atan2( x * cos(rotx), z )

	head_set(roty,-rotx)
	# head_set(pan,tilt)
	

def head_set(pan,tilt):
	# global gripper_pub
	# gripper_pub.publish(1.0)
	global head_pan_pub
	global head_tilt_pub
	print "pan: %f" % (pan)
	print "tilt: %f" % (tilt)
	head_pan_pub.publish(pan)
	head_tilt_pub.publish(tilt)

def gripper_set(val):
	global gripper_pub
	gripper_pub.publish((1-val)*-2.53)

def gripper_stop():
	global gripper_pub
	
	# gripper_pub.publish((1-val)*-2.53)

	# cs = robot.get_current_state()

def publish_arm_target(target_pos):
	global arm_target_marker_pub
	
	marker = Marker()
	# marker.header.frame_id = "/base_link"
	marker.header.frame_id = "/camera_rgb_optical_frame"
	# marker.header.frame_id = "/arm_target_frame"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.02
	marker.scale.y = 0.02
	marker.scale.z = 0.02
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 0.0
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0

	calibration_scale = 1.1
	marker.pose.position = Point(target_pos[0],target_pos[1],target_pos[2])
	# marker.pose.position = Point(target_pos[0]*calibration_scale,target_pos[1]*calibration_scale,target_pos[2])

	print marker.pose.position
	arm_target_marker_pub.publish(marker)

def publish_marker(pos):
	global marker_pub
	
	marker = Marker()
	# marker.header.frame_id = "/base_link"
	marker.header.frame_id = "/head_base_link"
	marker.type = marker.SPHERE
	marker.action = marker.ADD
	marker.scale.x = 0.02
	marker.scale.y = 0.02
	marker.scale.z = 0.02
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0
	marker.pose.orientation.w = 1.0

	marker.pose.position = pos

	# print marker.pose.position
	marker_pub.publish(marker)

def head_callback(data):
	global lowerHeadRad
	if(data.x == -1.0):
		head_set(data.y,data.z)
		return

	headPt = PointStamped()
	headPt.header.frame_id = "/head_base_link"
	headPt.header.stamp = rospy.Time.now()
	headPt.point = Point(-data.x,data.y,data.z)

	publish_marker(headPt.point)

	head_set_xyz(headPt)

	# basePt = tf.transformPoint("/base_link", headPt)

	# print "############# basePt #############"
	# print basePt
	# target_pos = [basePt.point.x,basePt.point.y,basePt.point.z]

	# Do target processing here:
	
	# marker.pose.position.x = target_pos[0]
	# marker.pose.position.y = target_pos[1]
	# marker.pose.position.z = target_pos[2]
	# marker.pose.position.x = back_target_pos[0]
	# marker.pose.position.y = back_target_pos[1]
	# marker.pose.position.z = back_target_pos[2]

	# marker.pose.position.x = math.cos(count / 50.0)
	# marker.pose.position.y = math.cos(count / 40.0) 
	# marker.pose.position.z = math.cos(count / 30.0)


def pos_callback(data):
	global marker_pub
	if(data.x == 0.0 and data.y == 0.0 and data.z == 0.0):
		return
	elif(data.x == -1.0 and data.y == -1.0):
		print "################################################"
		print "###################  Gripping!  ###################"
		print "################################################"

		if(data.z == -1.0):
			gripper_stop()
		else:
			gripper_set(data.z)
	
	elif(data.x == -1.0 and data.y == -2.0):
		orientation_pub.publish(data)	

	else:
		rospy.loginfo("Received a target_pos message: [%f, %f, %f]"%(data.x, data.y, data.z))

		scale = 3000.0
		scale = 1.0
		
		shoulder_lift_pos = [0.1, 0.03, 0.44]

		# target_pos = [data.z/scale, data.y/scale, data.x/scale]
		target_pos = [data.x, -data.y, data.z]

		# target_pos = [target_pos[0] + shoulder_lift_pos[0], 
		# target_pos[1] + shoulder_lift_pos[1], 
		# target_pos[2] + shoulder_lift_pos[2], ]

		# back_target_pos = [-data.z/scale, -data.y/scale, data.x/scale]

		# back_target_pos = [back_target_pos[0] + shoulder_lift_pos[0], 
		# back_target_pos[1] + shoulder_lift_pos[1], 
		# back_target_pos[2] + shoulder_lift_pos[2], ]

		# print "############# target_pos #############"
		# print target_pos

		# kinectPt = PointStamped()
		# kinectPt.header.frame_id = "/camera_rgb_optical_frame"
		# kinectPt.header.stamp = rospy.Time.now()
		# kinectPt.point = Point(target_pos[0],target_pos[1],target_pos[2])

		# kinectPt.header.stamp -= rospy.Duration(.22)
		# listener.waitForTransform("/camera_rgb_optical_frame", "/base_link", kinectPt.header.stamp, rospy.Duration(4.0))
		# basePt = tf.transformPoint("/base_link", kinectPt)

		# print "############# basePt #############"
		# print basePt
		# target_pos = [basePt.point.x,basePt.point.y,basePt.point.z]

		publish_arm_target(target_pos)

		# # Do target processing here:
		# marker = Marker()
		# # marker.header.frame_id = "/base_link"
		# marker.header.frame_id = "/camera_rgb_optical_frame"
		# marker.type = marker.SPHERE
		# marker.action = marker.ADD
		# marker.scale.x = 0.03
		# marker.scale.y = 0.03
		# marker.scale.z = 0.03
		# marker.color.a = 1.0
		# marker.color.r = 0.0
		# marker.color.g = 1.0
		# marker.color.b = 1.0
		# marker.pose.orientation.w = 1.0

		# marker.pose.position.x = target_pos[0]
		# marker.pose.position.y = target_pos[1]
		# marker.pose.position.z = target_pos[2]

		# marker_pub.publish(marker)

		# marker.pose.position.x = back_target_pos[0]
		# marker.pose.position.y = back_target_pos[1]
		# marker.pose.position.z = back_target_pos[2]

		# marker.pose.position.x = math.cos(count / 50.0)
		# marker.pose.position.y = math.cos(count / 40.0) 
		# marker.pose.position.z = math.cos(count / 30.0)


if __name__ == "__main__":
	Server()