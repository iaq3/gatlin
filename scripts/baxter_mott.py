#!/usr/bin/env python
import time
from threading import Thread, Lock
import rospy, sys, tf
from math import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from tf.transformations import *
from tf import *
from copy import deepcopy
from gatlin.msg import *
from gatlin.srv import *
from config import *


def distance(v1,v2):
	return np.linalg.norm(vector3_to_numpy(v1) - vector3_to_numpy(v2))

def length(v):
	return math.sqrt(np.dot(v, v))

def angle(v1, v2):
	return math.acos(np.dot(v1, v2) / (length(v1) * length(v2)))

class DynamicPose:
	def __init__(self):
		# self.FIXED_FRAME = "global_map"
		self.FIXED_FRAME = "base"
		self.tfl = tf.TransformListener()
		self.pose_sub = None
		self.reset()

	def subscribe(self, topic, dps):
		self.reset()
		dps.append((topic, self))
		self.topic = topic

	def reset(self):
		if self.pose_sub:
			self.pose_sub.unregister()
		self.ps = PoseStamped()
		self.last_update = 0
		self.pose_sub = None
		self.topic = ""

	def set_pose(self, ps):
		self.ps = self.transform_pose(self.FIXED_FRAME, ps)
		self.last_update = time.time()

	# transform the pose stamped to the new frame
	def transform_pose(self, new_frame, pose):
		if pose.header.frame_id == new_frame:
			return pose
		try:
			ps = deepcopy(pose)
			ps.header.stamp = rospy.Time(0)
			self.tfl.waitForTransform(ps.header.frame_id, new_frame, rospy.Time(0), rospy.Duration(4.0))
			new_pose = self.tfl.transformPose(new_frame, ps)
			new_pose.header.stamp = deepcopy(pose.header.stamp)
			return new_pose
		except Exception as e:
			rospy.logerr(e)
			rospy.logerr("no transform")
			return None

class Nav_Manip_Controller :

	def grabObject(self, dynamic_pose) :
		#holding_object = False
		#while not holding_object :

		if self.command_state == self.CANCELLED :
			return
		self.pauseCommand()

		self.publishResponse("Attempting to grab "+dynamic_pose.topic)

		resp = self.move_arm(OPEN_GRIPPER, PoseStamped())

		# rospy.logerr(dynamic_pose.ps)
		base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)
		#base_pose.pose.position.z -= .025 #??? maybe not best practice
		resp = self.move_arm(MOVE_TO_POS, base_pose)

		if not resp.success:
			rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
			# try moving to it again
			#self.servoBaseToDynamicPos(self.object_pose)
			#TODO check this
			rospy.sleep(1)

		self.pauseCommand()

		resp = self.move_arm(CLOSE_GRIPPER, PoseStamped())

		self.pauseCommand()

		#resp = self.move_arm(RESET_ARM, PoseStamped())
		# base_pose.pose.position.z += .1
		# resp = self.move_arm(MOVE_TO_POS, base_pose)
		# if not resp.success:
		# 	rospy.logerr("RESET_ARM FAILED")

		# no object detection in last second, it is likely in robot's hand
		#if time.time() - dynamic_pose.last_update > 1 :
		#holding_object = True

		self.publishResponse("Grabbed "+dynamic_pose.topic)

	def pauseCommand(self) :
		while self.command_state == self.PAUSING :
			rospy.sleep(.03)

	def releaseObject(self, dynamic_pose) :
		if self.command_state == self.CANCELLED :
			return

		self.pauseCommand()

		self.publishResponse("Releasing object to "+dynamic_pose.topic)
		base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)
		resp = self.move_arm(MOVE_TO_POS, base_pose)

		self.pauseCommand()

		if not resp.success:
				rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
		resp = self.move_arm(OPEN_GRIPPER, PoseStamped())

		self.pauseCommand()
		base_pose.pose.position.z += .1
		resp = self.move_arm(MOVE_TO_POSE, base_pose)

	def interActionDelay(self, delay) : #if user tells command to quit, then you don't want delays to stack
	 	if self.command_state == self.RUNNING :
			rospy.sleep(delay)

	def run_mott_sequence(self) :
		self.grabObject(self.object_pose)
		
		self.interActionDelay(1)

		self.releaseObject(self.target_pose)

		if self.command_state == self.RUNNING :
			self.publishResponse("finished mott") #string must contain finished
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished mott while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command") 

	def MottCallback(self, data) :
		rospy.logerr(data)

		self.command_state = self.RUNNING

		if data.object_pose_topic != "" :
			self.object_pose.subscribe(data.object_pose_topic, self.dynamic_poses)

		if data.target_pose_topic != "" :
			self.target_pose.subscribe(data.target_pose_topic, self.dynamic_poses)
		
		if (data.object_pose) :
			self.object_pose.set_pose(data.object_pose)

		if (data.target_pose) :
			self.target_pose.set_pose(data.target_pose)

		if data.command == "mott" :
			self.run_mott_sequence()


	def MottCommandCallback(self, data) :
		print "received "+data.data
		data.data = data.data.lower()
		if "cancel" in data.data :
			print "State = cancelling action"
			self.command_state = self.CANCELLED
		elif "paus" in data.data :
			print "state = pausing"
			self.command_state = self.PAUSING
		elif "run" in data.data :
			print "state = running"
			self.command_state = self.RUNNING

	def publishResponse(self, statement) :
		rospy.loginfo(statement)
		self.response_pub.publish(statement)

	# get the distance from the base to the given pose
	def distanceToPose(self, ps) :
		origin = Point(0,0,0)
		pose = self.transform_pose(self.BASE_FAME, ps)
		return distance(origin, pose.pose.position)

	# transform the pose stamped to the new frame
	def transform_pose(self, new_frame, ps):
		if ps.header.frame_id == new_frame:
			return ps
		try:
			temp_ps = deepcopy(ps)
			temp_ps.header.stamp = rospy.Time(0)
			self.tfl.waitForTransform(temp_ps.header.frame_id, new_frame, rospy.Time(0), rospy.Duration(4.0))
			new_pose = self.tfl.transformPose(new_frame, temp_ps)
			new_pose.header.stamp = deepcopy(pose.header.stamp)
			return new_pose
		except Exception as e:
			rospy.logerr(e)
			rospy.logerr("no transform")
			return None

	def objectListCallback(self, ol):
		# rospy.logerr(ol)
		for obj in ol.objects:
			for (topic, dp) in self.dynamic_poses:
				if "%s_%s" % (obj.color, obj.id) == topic:
					dp.set_pose(obj.pose)
					# rospy.logerr(dp.ps)
					# rospy.logerr(obj.pose)


	def __init__(self):
		limb = "left"#TODO change to param or sumthing
		self.limb = limb

		self.robot_name = "baxter_"+limb
		rospy.init_node('%s_nav_manip_controller'%self.robot_name)

		#self.robot_pose = DynamicPose()
		#self.robot_pose.subscribe("/robot_pose") #TODO change to end effector position??

		self.object_pose = DynamicPose()
		self.target_pose = DynamicPose()

		self.dynamic_poses = []
		rospy.Subscriber("/%s/ar_marker_list" % self.robot_name, ObjectList, self.objectListCallback, queue_size=3)
		self.object_pose.subscribe("ar_8", self.dynamic_poses)

		self.RUNNING = 0
		self.PAUSING = 1
		self.CANCELLED = 2
		self.command_state = 0 #keeps track of running, pausing, cancelled

		#self.FIXED_FRAME = "global_map"
		self.BASE_FAME = "base"#TODO look into

		robot = "baxter_" + limb
		self.response_pub = rospy.Publisher("/"+robot+"_mott_response", String, queue_size = 1)
		self.baxter_cmd_pub = rospy.Publisher("/"+robot+"_cmd", Int32, queue_size = 1)

		rospy.Subscriber("/"+robot+"_mott", Mott, self.MottCallback, queue_size = 1)
		rospy.Subscriber("/"+robot+"_mott_command", String, self.MottCommandCallback, queue_size = 1)

		# self.move_arm = createServiceProxy("baxter/move/arm", MoveRobot, self.robot_name)
		self.move_arm = createServiceProxy(robot+"/move_robot", MoveRobot)

		self.test_pose_pub = rospy.Publisher('/test_obj_pose', PoseStamped, queue_size = 1)

		self.tfl = tf.TransformListener()

		rospy.spin()

if __name__ == "__main__":
	Nav_Manip_Controller()