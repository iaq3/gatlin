#!/usr/bin/env python
# Names: Zach Vinegar, Isaac Qureshi
from config import *
from Dynamic import *
from MRCTester import *
from CmdReqQueue import *
import numpy as np
import rospy
from threading import Thread
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *
import networkx as nx
import matplotlib.pyplot as plt
import tf
from rospy_message_converter import json_message_converter

class MRCTester:
	def __init__(self, mrc):
		self.mrc = mrc

	def test_mott_throw(self):
		table_z = -.230
		hp2_z = -0.548
		block_width = .0385

		crl = CommandRequestList()

		m = Mott()
		m.command = "mott"
		m.object_pose_topic = "ar_5"
		m.target_pose_topic = "target_1"

		m.object_pose.header.frame_id = "baxter"
		
		m.target_pose.header.frame_id = "baxter"
		m.target_pose.header.stamp = rospy.Time.now()
		m.target_pose.pose.position = Point(.6,-.5, table_z)
		m.target_pose.pose.orientation = Quaternion(0,1,0,0)

		mott_json = json_message_converter.convert_ros_message_to_json(m)

		crl = CommandRequestList()

		cr = CommandRequest()
		cr.id = "1"
		cr.action = "mott"
		cr.args = mott_json
		crl.commands.append(cr)

		m = Mott()
		m.command = "throw"
		m.object_pose_topic = "ar_5"

		mott_json = json_message_converter.convert_ros_message_to_json(m)

		cr = CommandRequest()
		cr.id = "2"
		cr.action = "throw"
		cr.args = mott_json
		crl.commands.append(cr)

		crl.parents = ["1"]
		crl.children = ["2"]

		self.mrc.mr_command_req_callback(crl)

	def one_block_move(self):
		m = Mott()
		m.command = "mott"
		m.object_pose_topic = "ar_5"
		m.target_pose_topic = "target_1"

		# m.object_pose = PoseStamped()

		table_z = -.230
		hp2_z = -0.548
		# m.object_pose.header.frame_id = "baxter"
		# m.object_pose.header.stamp = rospy.Time.now()
		# m.object_pose.pose.position = Point(.6,-.5, table_z)
		# m.object_pose.pose.position = Point(-0.1,0.725, hp2_z)
		# m.object_pose.pose.orientation = Quaternion(0,1,0,0)
		
		m.target_pose.header.frame_id = "baxter"
		m.target_pose.header.stamp = rospy.Time.now()
		# m.target_pose.pose.position = Point(.6,-.5, table_z)
		m.target_pose.pose.position = Point(0.1,0.725, hp2_z)
		m.target_pose.pose.orientation = Quaternion(0,1,0,0)
		# rospy.logerr(m)

		# m.object_pose.header.frame_id = "gatlin"
		# m.object_pose.header.stamp = rospy.Time.now()
		# m.object_pose.pose.position = Point(0.21,-0.37, 0.03)
		# m.object_pose.pose.orientation = Quaternion(0,1,0,0)
		mott_json = json_message_converter.convert_ros_message_to_json(m)

		# test CommandRequestList
		crl = CommandRequestList()
		cr = CommandRequest()
		cr.id = 1
		cr.action = "mott"
		cr.args = mott_json
		crl.commands.append(cr)

		self.mrc.mr_command_req_callback(crl)

	def two_block_stack(self):
		table_z = -.230
		hp2_z = -0.548
		block_width = .0385

		crl = CommandRequestList()

		m = Mott()
		m.command = "mott"
		m.object_pose_topic = "ar_3"
		m.target_pose_topic = "target_1"

		m.object_pose.header.frame_id = "baxter"
		
		m.target_pose.header.frame_id = "baxter"
		m.target_pose.header.stamp = rospy.Time.now()
		m.target_pose.pose.position = Point(.6,-.5, table_z)
		# m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
		m.target_pose.pose.orientation = Quaternion(0,1,0,0)

		mott_json = json_message_converter.convert_ros_message_to_json(m)

		cr = CommandRequest()
		cr.id = "1"
		cr.action = "mott"
		cr.args = mott_json
		crl.commands.append(cr)

		m = Mott()
		m.command = "mott"
		m.object_pose_topic = "ar_5"
		m.target_pose_topic = "target_2"

		m.object_pose.header.frame_id = "baxter"
		
		m.target_pose.header.frame_id = "baxter"
		m.target_pose.header.stamp = rospy.Time.now()
		m.target_pose.pose.position = Point(.6,-.5, table_z+block_width)
		# m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
		m.target_pose.pose.orientation = Quaternion(0,1,0,0)
		
		mott_json = json_message_converter.convert_ros_message_to_json(m)

		cr = CommandRequest()
		cr.id = "2"
		cr.action = "mott"
		cr.args = mott_json
		crl.commands.append(cr)

		crl.parents = ["1"]
		crl.children = ["2"]

		# test CommandRequestList
		self.mrc.mr_command_req_callback(crl)

	def three_block_stack(self):
		table_z = -.230
		hp2_z = -0.548
		block_width = .0385

		crl = CommandRequestList()

		m = Mott()
		m.command = "mott"
		m.object_pose_topic = "ar_5"
		m.target_pose_topic = "target_1"

		m.object_pose.header.frame_id = "baxter"
		
		m.target_pose.header.frame_id = "baxter"
		m.target_pose.header.stamp = rospy.Time.now()
		m.target_pose.pose.position = Point(.6,-.5, table_z+block_width*0)
		# m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
		m.target_pose.pose.orientation = Quaternion(0,1,0,0)

		mott_json = json_message_converter.convert_ros_message_to_json(m)

		cr = CommandRequest()
		cr.id = "1"
		cr.action = "mott"
		cr.args = mott_json
		crl.commands.append(cr)

		m = Mott()
		m.command = "mott"
		m.object_pose_topic = "ar_8"
		m.target_pose_topic = "target_2"

		m.object_pose.header.frame_id = "baxter"
		
		m.target_pose.header.frame_id = "baxter"
		m.target_pose.header.stamp = rospy.Time.now()
		m.target_pose.pose.position = Point(.6,-.5, table_z+block_width*1)
		# m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
		m.target_pose.pose.orientation = Quaternion(0,1,0,0)
		
		mott_json = json_message_converter.convert_ros_message_to_json(m)

		cr = CommandRequest()
		cr.id = "2"
		cr.action = "mott"
		cr.args = mott_json
		crl.commands.append(cr)

		m = Mott()
		m.command = "mott"
		m.object_pose_topic = "ar_3"
		m.target_pose_topic = "target_3"

		m.object_pose.header.frame_id = "baxter"
		
		m.target_pose.header.frame_id = "baxter"
		m.target_pose.header.stamp = rospy.Time.now()
		m.target_pose.pose.position = Point(.6,-.5, table_z+block_width*2)
		# m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
		m.target_pose.pose.orientation = Quaternion(0,1,0,0)
		
		mott_json = json_message_converter.convert_ros_message_to_json(m)

		cr = CommandRequest()
		cr.id = "3"
		cr.action = "mott"
		cr.args = mott_json
		crl.commands.append(cr)

		crl.parents = ["1","2"]
		crl.children = ["2","3"]

		# test CommandRequestList
		self.mrc.mr_command_req_callback(crl)