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

class DynamicObject:
	def __init__(self):
		self.FIXED_FRAME = "odom"
		self.tfl = tf.TransformListener()
		self.reset()

	def subscribe(self, topic):
		self.reset()
		params = topic.split("_")
		self.color = params[0]
		self.id = params[1]

	def reset(self):
		self.ps = PoseStamped()
		self.last_update = 0
		self.id = ""
		self.color = ""

	def set_pose(self, ps):
		self.ps = self.transform_pose(self.FIXED_FRAME, ps)
		self.last_update = time.time()

	def update_pose(self, objectlist):
		for obj in objectlist.objects:
			if obj.color == self.color and obj.id == self.id:
				self.ps = self.transform_pose(self.FIXED_FRAME, obj.pose)

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

class DynamicPose:
	def __init__(self):
		self.FIXED_FRAME = "odom"
		self.tfl = tf.TransformListener()
		self.pose_sub = None
		self.reset()

	def subscribe(self, topic):
		self.reset()
		self.pose_sub = rospy.Subscriber(topic, PoseStamped, self.set_pose, queue_size = 1)
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

	def servo_base_to_pos(self, desired_pos, actual_pos) :
		desired_pos = vector3_to_numpy(desired_pos)
		actual_pos = vector3_to_numpy(actual_pos)
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

	def moveBaseToDynamicPos(self, dynamic_pose) :
		if self.command_state == self.CANCELLED :
			return

		rate = rospy.Rate(30)
		goal_tolerence = .7

		if self.distanceToPose(dynamic_pose.ps) > goal_tolerence :
			self.publishResponse("Gmap base to "+dynamic_pose.topic)
			
			self.gmapBaseTo(dynamic_pose.ps)
			#now this contains logic to cancel, pause, and resume
			while self.distanceToPose(dynamic_pose.ps) > goal_tolerence : 
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
					self.gmapBaseTo(dynamic_pose.ps)
				rate.sleep()

			self.cancelgmapBaseTo()

	def servoBaseToDynamicPos(self, dynamic_pose) :
		if self.command_state == self.CANCELLED :
			return

		# self.publishResponse("Servo base to "+dynamic_pose.topic)

		rate = rospy.Rate(30)
		desired_pos = Point(.29,0,0)
		goal_tolerence = .025

		base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)
		while self.servo_base_to_pos(desired_pos, base_pose.pose.position) > goal_tolerence :
			if self.command_state == self.CANCELLED :
				return
			
			self.pauseCommand() #TODO move fr
			if self.command_state == self.RUNNING :
				base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)

			rate.sleep()

	def grabObject(self, dynamic_pose) :
		

		holding_object = False
		while not holding_object :
			if self.command_state == self.CANCELLED :
				return
			self.pauseCommand()

			# self.publishResponse("Attempting to grab "+dynamic_pose.topic)
			resp = self.move_arm(OPEN_GRIPPER, PoseStamped())

			base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)
			base_pose.pose.position.z -= .025 #??? maybe not best practice
			resp = self.move_arm(MOVE_TO_POSE_INTERMEDIATE, base_pose)
			if not resp.success:
				rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
				# try moving to it again
				self.servoBaseToDynamicPos(self.object)
				#TODO check this
				rospy.sleep(1)
				continue

			resp = self.move_arm(CLOSE_GRIPPER, PoseStamped())

			self.pauseCommand()

			resp = self.move_arm(RESET_ARM, PoseStamped())
			if not resp.success:
				rospy.logerr("RESET_ARM FAILED")

			# no object detection in last second, it is likely in robot's hand
			if time.time() - dynamic_pose.last_update > 1 :
				holding_object = True

		# self.publishResponse("Grabbed "+dynamic_pose.topic)

	def pauseCommand(self) :
		while self.command_state == self.PAUSING :
			rospy.sleep(.03)

	def releaseObject(self, dynamic_pose) :
		if self.command_state == self.CANCELLED :
			return

		self.pauseCommand()

		# self.publishResponse("Releasing object to "+dynamic_pose.topic)
		base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)
		resp = self.move_arm(MOVE_TO_POSE_INTERMEDIATE, base_pose)
		if not resp.success:
				rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
		resp = self.move_arm(OPEN_GRIPPER, PoseStamped())

		self.pauseCommand()
		resp = self.move_arm(RESET_ARM, PoseStamped())

	def interActionDelay(self, delay) : #if user tells command to quit, then you don't want delays to stack
	 	if self.command_state == self.RUNNING :
			rospy.sleep(delay)

	def run_mott_sequence(self) :


		self.moveBaseToDynamicPos(self.object)

		#if not self.joystick_topic == "" : TODO
		self.servoBaseToDynamicPos(self.object)

		self.interActionDelay(1)
		self.grabObject(self.object)
		
		self.interActionDelay(1)

		self.moveBaseToDynamicPos(self.target)
		self.servoBaseToDynamicPos(self.target)
		self.interActionDelay(1)
		self.releaseObject(self.target)

		if self.command_state == self.RUNNING :
			self.publishResponse("finished mott") #string must contain finished
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished mott while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command") 

	def base_to_sequence(self) :
		self.moveBaseToDynamicPos(self.target)
		self.servoBaseToDynamicPos(self.target)
		
		if self.command_state == self.RUNNING :
			self.publishResponse("finished moving base to target")
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished move base while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command") 
				

	def MottCallback(self, data) :
		if data.object_pose_topic != "" :
			self.object.subscribe(data.object_pose_topic)

		if data.target_pose_topic != "" :
			self.target.subscribe(data.target_pose_topic)
		
		if (data.object_pose) :
			self.object.set_pose(data.object_pose)

		if (data.target_pose) :
			self.target.set_pose(data.target_pose)

		if data.command == "mott" :
			self.run_mott_sequence()
		elif data.command == "move_base" :
			print "Starting Move Base TO"
			self.base_to_sequence()

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

	def baseJoystickPublish (msg) :
		self.base_joystick_pub.publish(msg)

	def cancelgmapBaseTo(self) :
		self.gatlin_cmd_pub.publish(9)

	def gmapBaseTo(self, ps) :
		map_target_pose = self.transform_pose("map", ps)
		map_robot_pose = self.transform_pose("map", self.robot_pose.ps)
		map_target_pose.pose.orientation = map_robot_pose.pose.orientation
		self.gmap_base_pub.publish(map_target_pose)

	def publishResponse(self, statement) :
		rospy.loginfo(statement)
		self.response_pub.publish(statement)

	# get the distance from the base to the given pose
	def distanceToPose(self, ps) :
		origin = Point(0,0,0)
		pose = self.transform_pose(self.BASE_FAME, ps)
		return distance(origin, pose.pose.position)

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

	def set_object_poses(self, objectlist):
		self.object.update_pose(objectlist)
		self.target.update_pose(objectlist)

	def __init__(self):
		self.robot_name = "gatlin"



		rospy.init_node('%s_nav_manip_controller'%self.robot_name)

		self.robot_pose = DynamicPose()
		self.robot_pose.subscribe("/robot_pose")

		self.object = DynamicObject()
		self.target = DynamicObject()

		self.objectlist_sub = rospy.Subscriber("/gatlin/objectlist", ObjectList, self.set_object_poses, queue_size = 3)

		self.RUNNING = 0
		self.PAUSING = 1
		self.CANCELLED = 2
		self.command_state = 0 #keeps track of running, pausing, cancelled

		self.FIXED_FRAME = "odom"
		self.BASE_FAME = "base_link"

		
		''' 
		TODO
		add 
		self.joystick_topic = "/cmd_vel_mux/input/teleop"

		'''


		self.base_joystick_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist) #<--- joystick topic
		self.response_pub = rospy.Publisher("/gatlin_mott_response", String)
		self.gmap_base_pub = rospy.Publisher("/move_to_goal", PoseStamped)
		self.gatlin_cmd_pub = rospy.Publisher("/gatlin_cmd", Int32)
		self.base_joystick_pub = rospy.Publisher("/cmd_vel_mux/input/teleop" , Twist)

		rospy.Subscriber("/gatlin_mott", Mott, self.MottCallback, queue_size = 1)
		rospy.Subscriber("/gatlin_mott_command", String, self.MottCommandCallback, queue_size = 1)

		self.move_arm = createServiceProxy("move/arm", MoveRobot, self.robot_name)

		self.test_pose_pub = rospy.Publisher('/test_obj_pose', PoseStamped)

		self.tfl = tf.TransformListener()

		rospy.spin()

if __name__ == "__main__":
	Nav_Manip_Controller()