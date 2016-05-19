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
from Dynamic import *

from move_base_msgs.msg import *
import actionlib
from actionlib_msgs.msg import *

def distance(v1,v2):
	return np.linalg.norm(vector3_to_numpy(v1) - vector3_to_numpy(v2))

def length(v):
	return math.sqrt(np.dot(v, v))

def angle(v1, v2):
	return math.acos(np.dot(v1, v2) / (length(v1) * length(v2)))

class Nav_Manip_Controller :

	def servo_base_to_pose(self, desired_pose, actual_pose) :
		desired_pos = vector3_to_numpy(Vector3(0,0,0))
		actual_pos = vector3_to_numpy(actual_pose.position)
		actual_pos[2] = 0

		error_vec =  actual_pos - desired_pos
		error = np.linalg.norm(error_vec)

		forward = error_vec[0]
		turn = error_vec[1]

		error_angle = angle(error_vec, desired_pos)
		# rospy.logerr(error_angle)
		if error_angle > 0.25  and error_angle < pi - 0.25: #???
			forward = 0

		maxVel = .07
		minVel = .05
		mag = (turn**2 + forward**2)**.5
		if (mag > maxVel) :
			turn = (turn/mag) * maxVel
			forward = (forward/mag) * maxVel

		if (mag < minVel) :
			turn = (turn/mag) * minVel
			forward = (forward/mag) * minVel

		turn *= 1.7

		msg = Twist (Point(forward, 0.0, 0.0), Point(0.0, 0.0, turn))
		self.base_joystick_pub.publish(msg)

		return error

	def getOffsetPose(self, ps, offset_t, output_frame=None):
		if output_frame != None:
			ps = self.transform_pose(output_frame, ps)

		if offset_t == None:
			return ps

		t = transform_from_pose(ps.pose)
		t_to_offset_t = multiply_transforms(t, offset_t)
		ps_offset = deepcopy(ps)
		ps_offset.pose = transform_to_pose(t_to_offset_t)

		return ps_offset

	def moveBaseToDynamicPos(self, dynamic_pose, offset_t=None, goal_tolerence=0.2) :
		if self.command_state == self.CANCELLED :
			return

		resp = self.move_arm("RESET_ARM", PoseStamped())
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		base_offset_ps = self.getOffsetPose(dynamic_pose.ps, offset_t)

		if self.distanceToPose(base_offset_ps) > goal_tolerence :
			self.publishResponse("Gmap base to %s_%s" % (dynamic_pose.color, dynamic_pose.id))
			
			self.gmapBaseTo(base_offset_ps)
			#now this contains logic to cancel, pause, and resume
			dist = self.distanceToPose(base_offset_ps)
			rate = rospy.Rate(10)
			state = None
			start_time = time.time()
			duration = 0
			while not state == GoalStatus.SUCCEEDED and duration < 20:
				duration = time.time() - start_time
				state = self.move_base.get_state()
				if state == GoalStatus.SUCCEEDED:
					rospy.loginfo("move_base SUCCEEDED")


				base_offset_ps = self.getOffsetPose(dynamic_pose.ps, offset_t)
				dist = self.distanceToPose(base_offset_ps)
				rospy.logerr(dist)
				if self.command_state == self.CANCELLED :
					self.cancelgmapBaseTo()
					return
				paused = False
				while self.command_state == self.PAUSING :
					if not paused :
						paused = True
						self.cancelgmapBaseTo()
					rate.sleep()
				if paused and self.command_state == self.RUNNING:
					self.gmapBaseTo(base_offset_ps)
				rate.sleep()

			self.cancelgmapBaseTo()

	def gmapBaseTo(self, ps):
		goal = MoveBaseGoal()
		self.move_base_target = self.transform_pose("map", ps)
		goal.target_pose = self.move_base_target
		rospy.logerr(goal)
		self.move_base.send_goal(goal)

	def cancelgmapBaseTo(self) :
		rospy.logerr("Cancelling current action")
		self.move_base.cancel_goal()

	def wait_for_move_base(self):
		success = self.move_base.wait_for_result(rospy.Duration(60)) 

		if not success:
				self.move_base.cancel_goal()
				rospy.logerr("move_base FAILURE")
		else:
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("move_base SUCCEEDED")

	# def cancelgmapBaseTo(self) :
	# 	self.gatlin_cmd_pub.publish(9)

	# def gmapBaseTo(self, ps) :
	# 	map_target_pose = self.transform_pose("map", ps)
	# 	# map_robot_pose = self.transform_pose("map", self.robot_pose.ps)
	# 	# map_target_pose.pose.orientation = map_robot_pose.pose.orientation
	# 	self.gmap_base_pub.publish(map_target_pose)

	def servoBaseToDynamicPos(self, dynamic_pose, offset_t=None) :
		if self.command_state == self.CANCELLED :
			return

		resp = self.move_arm("RESET_ARM", PoseStamped())
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		self.publishResponse("Servo base to %s_%s" % (dynamic_pose.color, dynamic_pose.id))

		if dynamic_pose.color == "hp":
			rospy.logerr("MOVING TO HP")
			offset_t = Transform()
			offset_t.translation = Vector3(-.28,0,0)
			offset_t.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
			# desired_pos = Point(.28,0,0) # z is set to 0 when checking error
			resp = self.move_head("LOOK_FORWARD", PoseStamped())
			goal_tolerence = .035
		elif dynamic_pose.color == "basetarget":
			offset_t = Transform()
			offset_t.translation = Vector3(0,0,0)
			offset_t.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
			# desired_pos = Point(.30,0,0)
			resp = self.move_head("LOOK_DOWNWARD", PoseStamped())
			goal_tolerence = .05
		else:
			offset_t = Transform()
			offset_t.translation = Vector3(-.30,0,0)
			offset_t.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
			# desired_pos = Point(.30,0,0)
			resp = self.move_head("LOOK_DOWN", PoseStamped())
			goal_tolerence = .025

		base_offset_ps = self.getOffsetPose(dynamic_pose.ps, offset_t, output_frame=self.BASE_FRAME)
		desired_pose = Pose()
		# rot_tolerance = .07
		# base_pose = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)
		rate = rospy.Rate(30)
		while self.servo_base_to_pose(desired_pose, base_offset_ps.pose) > goal_tolerence :
			if self.command_state == self.CANCELLED :
				return
			
			self.pauseCommand() #TODO move fr
			if self.command_state == self.RUNNING :
				base_offset_ps = self.getOffsetPose(dynamic_pose.ps, offset_t, output_frame=self.BASE_FRAME)

			rate.sleep()

	def grabObject(self, dynamic_pose) :

		holding_object = False
		while not holding_object :
			if self.command_state == self.CANCELLED :
				return
			self.pauseCommand()

			self.publishResponse("Attempting to grab %s_%s" % (dynamic_pose.color, dynamic_pose.id))
			resp = self.move_arm("OPEN_GRIPPER", PoseStamped())

			offset_t = Transform()
			# offset_t.translation = Vector3(-0.093, -0.019, 0.005)
			offset_t.translation = Vector3(-0.028, 0.00, -0.025)
			offset_t.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
			base_offset_ps = self.getOffsetPose(dynamic_pose.ps, offset_t, output_frame=self.BASE_FRAME)

			resp = self.move_arm("MOVE_TO_POSE_INTERMEDIATE", base_offset_ps)

			if not resp.success:
				rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
				# try moving to it again
				self.servoBaseToDynamicPos(self.object_dp)
				#TODO check this
				rospy.sleep(1)
				continue

			resp = self.move_arm("CLOSE_GRIPPER", PoseStamped())

			self.pauseCommand()

			resp = self.move_arm("RESET_ARM", PoseStamped())
			if not resp.success:
				rospy.logerr("RESET_ARM FAILED")

			# no object detection in last second, it is likely in robot's hand
			since_update = time.time() - dynamic_pose.last_update
			if since_update > 1 :
				holding_object = True

		self.publishResponse("Grabbed %s_%s" % (dynamic_pose.color, dynamic_pose.id))

	def pauseCommand(self) :
		while self.command_state == self.PAUSING :
			rospy.sleep(.03)

	def releaseObject(self, dynamic_pose) :
		if self.command_state == self.CANCELLED :
			return

		self.pauseCommand()

		self.publishResponse("Releasing object to %s_%s" % (dynamic_pose.color, dynamic_pose.id))

		if dynamic_pose.color == "hp":
			resp = self.move_arm("PLACE_UPPER", PoseStamped())
			rospy.logerr("PLACE_UPPER")
		else:
			base_pose = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)
			resp = self.move_arm("MOVE_TO_POSE_INTERMEDIATE", base_pose)

		if not resp.success:
			rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
		resp = self.move_arm("OPEN_GRIPPER", PoseStamped())

		self.pauseCommand()
		resp = self.move_arm("RESET_ARM", PoseStamped())

	def interActionDelay(self, delay) : #if user tells command to quit, then you don't want delays to stack
		if self.command_state == self.RUNNING :
			rospy.sleep(delay)

	def run_mott_sequence(self) :
		resp = self.move_arm("OPEN_GRIPPER", PoseStamped())
		resp = self.move_arm("RESET_ARM", PoseStamped())
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		if self.object_dp.ps == None or self.object_dp.ps.pose.position == Point():
			self.search_sequence(self.object_dp)

		self.moveBaseToDynamicPos(self.object_dp)
		self.servoBaseToDynamicPos(self.object_dp)
		self.interActionDelay(1)

		self.grabObject(self.object_dp)
		self.interActionDelay(1)

		if self.target_dp.ps == None or self.target_dp.ps.pose.position == Point():
			self.search_sequence(self.target_dp)

		self.moveBaseToDynamicPos(self.target_dp)
		self.servoBaseToDynamicPos(self.target_dp)
		self.interActionDelay(1)

		self.releaseObject(self.target_dp)

		ol = ObjectList()
		o = Object()
		o.id = self.object_dp.id
		o.color = self.object_dp.color
		o.pose = PoseStamped()
		ol.objects.append(o)
		self.objectlist_pub.publish(ol)

		ol = ObjectList()
		o = Object()
		o.id = self.object_dp.id
		o.color = self.object_dp.color
		o.pose = deepcopy(self.target_dp.ps)
		ol.objects.append(o)
		self.objectlist_pub.publish(ol)
		self.objectlist_pub.publish(ol)

		if self.command_state == self.RUNNING :
			self.publishResponse("finished mott") #string must contain finished
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished mott while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command") 

	def base_to_sequence(self) :
		resp = self.move_arm("RESET_ARM", PoseStamped())
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		# rospy.logerr()

		# if self.target_dp.ps == None or self.target_dp.ps.pose.position == Point():
		# 	self.search_sequence(self.target_dp)

		self.moveBaseToDynamicPos(self.target_dp)
		self.servoBaseToDynamicPos(self.target_dp)
		
		if self.command_state == self.RUNNING :
			self.publishResponse("finished moving base to target")
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished move base while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command")

	def grab_sequence(self) :
		resp = self.move_arm("OPEN_GRIPPER", PoseStamped())
		resp = self.move_arm("RESET_ARM", PoseStamped())
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		if self.object_dp.ps.header.frame_id == "":
			self.search_sequence(self.object_dp)

		offset_t = Transform()
		offset_t.translation = Vector3(-.25, 0, 0.0)
		offset_t.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)

		# self.moveBaseToDynamicPos(self.object_dp, offset_t=offset_t)
		self.servoBaseToDynamicPos(self.object_dp)
		self.interActionDelay(1)

		self.grabObject(self.object_dp)
		self.interActionDelay(1)
		
		if self.command_state == self.RUNNING :
			self.publishResponse("finished grabbing")
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished grabbing while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command")

	def release_sequence(self) :
		if self.target_dp.ps == None or self.target_dp.ps.header.frame_id == "":
			ps = PoseStamped()
			ps.header.stamp = rospy.Time.now()
			ps.header.frame_id = "base_link"
			ps.pose.position = Point(.25,0,.03)
			ps.pose.orientation = Quaternion(0,1,0,0)
			self.target_dp.set_pose(ps)


		# self.moveBaseToDynamicPos(self.target_dp)
		# self.servoBaseToDynamicPos(self.target_dp)
		# self.interActionDelay(1)

		self.releaseObject(self.target_dp)

		ol = ObjectList()
		o = Object()
		o.id = self.object_dp.id
		o.color = self.object_dp.color
		o.pose = PoseStamped()
		ol.objects.append(o)
		self.objectlist_pub.publish(ol)

		ol = ObjectList()
		o = Object()
		o.id = self.object_dp.id
		o.color = self.object_dp.color
		o.pose = deepcopy(self.target_dp.ps)
		ol.objects.append(o)
		self.objectlist_pub.publish(ol)
		self.objectlist_pub.publish(ol)

		if self.command_state == self.RUNNING :
			self.publishResponse("finished mott") #string must contain finished
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished mott while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command") 

	def search_sequence(self, dp) :

		rospy.logerr("Searching for %s_%s" % (dp.color, dp.id))
		
		# resp = self.move_head("LOOK_DOWN", PoseStamped())

		ps = PoseStamped()
		ps.header.frame_id = "base_link"
		ps.header.stamp = rospy.Time.now()
		ps.pose.position = Point(0.30, 0.00, 0.30)

		while dp.ps == None or dp.ps.pose.position == Point():
			if ps.pose.position.z < 0:
				ps.pose.position.z = 0
			else:
				ps.pose.position.z -= .05
			resp = self.move_head("LOOK_AT", ps)
			rospy.logerr("%s_%s not found yet" % (dp.color, dp.id))
			rospy.sleep(1.5)

		rospy.logerr("%s_%s found!" % (dp.color, dp.id))
		# resp = self.move_arm("RESET_ARM", PoseStamped())
		# if not resp.success:
		# 	rospy.logerr("RESET_ARM FAILED")

		# self.moveBaseToDynamicPos(self.target_dp)
		# self.servoBaseToDynamicPos(self.target_dp)
		
		# if self.command_state == self.RUNNING :
		# 	self.publishResponse("finished moving base to target")
		# elif self.command_state == self.PAUSING :
		# 	self.publishResponse("finished move base while pausing!?!?") 
		# elif self.command_state == self.CANCELLED :
		# 	self.publishResponse("quitting on user command")

	def MottCallback(self, data) :

		self.dm.dynamic_poses = []
		self.object_dp = self.dm.create_dp(self.FIXED_FRAME)
		self.target_dp = self.dm.create_dp(self.FIXED_FRAME)

		self.command_state = self.RUNNING

		# rospy.logerr(data)

		if data.object_pose_topic != "" :
			self.object_dp.subscribe_name(data.object_pose_topic)

		if data.target_pose_topic != "" :
			self.target_dp.subscribe_name(data.target_pose_topic)
		
		if (data.object_pose) :
			self.object_dp.set_pose(data.object_pose)

		if (data.target_pose) :
			self.target_dp.set_pose(data.target_pose)
		
		rospy.sleep(.2)

		if data.command == "mott" :
			self.run_mott_sequence()
		elif data.command == "move_base" :
			rospy.loginfo("Starting Move Base TO")
			self.base_to_sequence()
		elif data.command == "grab" :
			rospy.loginfo("Starting Grab")
			self.grab_sequence()
		elif data.command == "release" :
			rospy.loginfo("Starting Release")
			self.release_sequence()
		elif data.command == "search" :
			rospy.loginfo("Starting Search")
			self.search_sequence()
		else:
			rospy.logerr("Invalid Command: %s" % data.command)

	def MottCommandCallback(self, data) :
		rospy.loginfo("received "+data.data)
		data.data = data.data.lower()
		if "cancel" in data.data :
			rospy.loginfo("State = cancelling action")
			self.command_state = self.CANCELLED
		elif "paus" in data.data :
			rospy.loginfo("state = pausing")
			self.command_state = self.PAUSING
		elif "run" in data.data :
			rospy.loginfo("state = running")
			self.command_state = self.RUNNING

	def baseJoystickPublish (msg) :
		self.base_joystick_pub.publish(msg)

	def publishResponse(self, statement) :
		rospy.loginfo(statement)
		self.response_pub.publish(statement)

	# get the distance from the base to the given pose
	def distanceToPose(self, ps) :
		origin = Point(0,0,0)
		pose = self.transform_pose(self.BASE_FRAME, ps)
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
			new_pose.header.stamp = deepcopy(ps.header.stamp)
			return new_pose
		except Exception as e:
			rospy.logerr(e)
			rospy.logerr("no transform")
			return None

	def __init__(self):
		rospy.init_node('gatlin_nav_manip_controller')

		# self.robot_pose = DynamicPose()
		# self.robot_pose.subscribe("/robot_pose")
		
		self.tfl = tf.TransformListener()

		# DynamicManager init
		self.dm = DynamicManager(self.tfl)
		self.dm.add_ol_sub("/gatlin/ar_marker_list")
		# self.dm.add_ol_sub("/server/ar_marker_list")

		self.RUNNING = 0
		self.PAUSING = 1
		self.CANCELLED = 2
		self.command_state = 0 #keeps track of running, pausing, cancelled

		# self.FIXED_FRAME = "global_map"
		self.FIXED_FRAME = "odom"
		self.BASE_FRAME = "base_link"

		robot = "gatlin"
		self.base_joystick_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist) #<--- joystick topic
		self.gmap_base_pub = rospy.Publisher("/move_to_goal", PoseStamped)

		self.response_pub = rospy.Publisher("/%s_mott_response" % robot, String, queue_size = 1)
		self.baxter_cmd_pub = rospy.Publisher("/%s_cmd" % robot, Int32, queue_size = 1)

		rospy.Subscriber("/%s_mott" % robot, Mott, self.MottCallback, queue_size = 1)
		rospy.Subscriber("/%s_mott_command" % robot, String, self.MottCommandCallback, queue_size = 1)

		self.move_arm = createServiceProxy("%s/move/arm" % robot, MoveRobot)
		self.move_head = createServiceProxy("%s/move/head" % robot, MoveRobot)

		self.objectlist_pub = rospy.Publisher("/%s/ar_marker_list" % robot, ObjectList, queue_size=3)

		self.test_pose_pub = rospy.Publisher('/test_obj_pose', PoseStamped)

		#tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("wait for the action server to come up")
		#allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(10))

		# # test move_base
		# ps = PoseStamped()
		# ps.header.stamp = rospy.Time.now()
		# ps.header.frame_id = "base_link"
		# ps.pose.position = Point(.1, 0, 0)
		# ps.pose.orientation = Quaternion(0,0,0,1)
		# self.gmapBaseTo(ps)
		# rospy.logerr("test move_base")

		rospy.spin()

if __name__ == "__main__":
	Nav_Manip_Controller()