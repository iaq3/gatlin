#!/usr/bin/env python
import time
from threading import Thread, Lock
import rospy, sys, tf, time
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

def distance(v1,v2):
	return np.linalg.norm(vector3_to_numpy(v1) - vector3_to_numpy(v2))

def length(v):
	return math.sqrt(np.dot(v, v))

def angle(v1, v2):
	return math.acos(np.dot(v1, v2) / (length(v1) * length(v2)))

class Nav_Manip_Controller :

	def make_map(self, left_min, left_max, right_min, right_max): 
		# Figure out how 'wide' each range is  
		leftSpan = left_max - left_min  
		rightSpan = right_max - right_min  

		# Compute the scale factor between left and right values 
		scaleFactor = float(rightSpan) / float(leftSpan) 

		# create interpolation function using pre-calculated scaleFactor
		def map_fn(value):
			return right_min + (value-left_min)*scaleFactor

		return map_fn

	def same_for_secs(self, val, dur, thresh):
		t = time.time()
		self.same_vals.append(val)
		self.same_val_times.append(t)
		while t - self.same_val_times[0] > dur:
			self.same_vals.pop(0)
			self.same_val_times.pop(0)
		# rospy.logerr(len(self.same_vals))

		val_range = max(self.same_vals) - min(self.same_vals)
		time_range = max(self.same_val_times) - min(self.same_val_times)
		# rospy.logerr(type(time_range))

		return val_range < thresh and time_range > dur*.9

	def reset_same_for_secs(self):
		self.same_vals = []
		self.same_val_times = []

	def servo_base_to_pose(self, desired_pose, actual_pose, pos_tolerance, rot_tolerance, at_pos, at_rot, rotate) :
		# desired_pos = vector3_to_numpy(desired_pos)
		desired_pos = vector3_to_numpy(Vector3(0,0,0))
		actual_pos = vector3_to_numpy(actual_pose.position)
		actual_pos[2] = 0

		error_vec =  actual_pos - desired_pos
		pos_error = np.linalg.norm(error_vec)

		forward = error_vec[0]
		turn = error_vec[1]

		error_angle = angle(error_vec, desired_pos)
		# rospy.logerr(error_angle)
		just_turn_thresh = 0.20
		if error_angle > just_turn_thresh  and error_angle < pi - just_turn_thresh:
			forward = 0

		rospy.logerr(error_vec)
		rospy.logerr(pos_error)
		desired_rot = euler_from_quaternion([0,0,0,1])
		q = actual_pose.orientation
		actual_rot = euler_from_quaternion([q.x,q.y,q.z,q.w])

		desired_rot = np.array(desired_rot)
		actual_rot = np.array(actual_rot)
		rot_error_vec =  actual_rot - desired_rot
		# rospy.logerr(rot_error_vec)
		rot_error = rot_error_vec[2]
		rot_error = abs(rot_error_vec[2])

		if rotate:
			if pos_error < pos_tolerance or at_pos:
				turn = rot_error_vec[2]/2
				forward = 0
		else:
			at_rot = True

		if pos_error < pos_tolerance*3 or at_pos:
			if self.same_for_secs(pos_error, 5, 1.0):
				rospy.logerr("################# same_for_secs #################")
				at_pos = True
		# if point_at_obj_on_finish:
		# 	rot_error = 

		maxVel = .09
		minVel = .06
		mag = (turn**2 + forward**2)**.5
		if (mag > maxVel) :
			turn = (turn/mag) * maxVel
			forward = (forward/mag) * maxVel

		if (mag < minVel) :
			turn = (turn/mag) * minVel
			forward = (forward/mag) * minVel

		turn *= 1.7


		rospy.logerr("forward, turn")
		rospy.logerr((forward, turn))

		msg = Twist (Point(forward, 0.0, 0.0), Point(0.0, 0.0, turn))
		self.base_joystick_pub.publish(msg)

		return pos_error < pos_tolerance or at_pos, rot_error < rot_tolerance or at_rot

	def moveBaseToDynamicPos(self, dynamic_pose) :
		if self.command_state == self.CANCELLED :
			return

		resp = self.move_arm("RESET_ARM", PoseStamped())
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		rate = rospy.Rate(30)
		goal_tolerance = 5.0

		if self.distanceToPose(dynamic_pose.ps) > goal_tolerance :
			self.publishResponse("Gmap base to %s_%s" % (dynamic_pose.color, dynamic_pose.id))
			
			self.gmapBaseTo(dynamic_pose.ps)
			#now this contains logic to cancel, pause, and resume
			while self.distanceToPose(dynamic_pose.ps) > goal_tolerance : 
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

	def servoBaseToDynamicPose(self, dynamic_pose, dist, pos_tolerance, yoff=0.0, rotate=True) :
		if self.command_state == self.CANCELLED :
			return

		resp = self.move_arm("RESET_ARM", PoseStamped())
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		self.publishResponse("Servo base to %s_%s" % (dynamic_pose.color, dynamic_pose.id))

		rate = rospy.Rate(30)
		if dynamic_pose.color == "hp":
			rospy.logerr("MOVING TO HP")
			desired_pos = Point(.28,0,0) # z is set to 0 when checking error
			resp = self.move_head("LOOK_FORWARD", PoseStamped())
			pos_tolerance = .035
		else:
			desired_pos = Point(.30,0,0)
			resp = self.move_head("LOOK_DOWN", PoseStamped())
			# pos_tolerance = .025


		def getOffsetPose():
			# rospy.logerr(dynamic_pose.ps)
			base_pose = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)
			base_to_ar_t = transform_from_pose(base_pose.pose)

			offset_t = Transform()
			offset_t.translation = Vector3(-dist, yoff, 0.0)
			offset_t.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)

			base_to_offset_t = multiply_transforms(base_to_ar_t, offset_t)
			base_pose_offset = deepcopy(base_pose)
			base_pose_offset.pose = transform_to_pose(base_to_offset_t)

			return base_pose_offset

		# base_pose_offset = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)
		base_pose_offset = getOffsetPose()
		desired_pose = Pose()
		rot_tolerance = .07
		# pos_tolerance = .045
		# base_pose = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)
		at_pos, at_rot = False, False
		self.reset_same_for_secs()
		while not (at_pos and at_rot):
			# since_update = time.time() - dynamic_pose.last_update
			# if since_update > 1:
			# 	rospy.logerr("not seen for %.2f secs" % since_update)
			# 	continue
			# rospy.logerr((at_pos, at_rot))
			at_pos, at_rot = self.servo_base_to_pose(
				desired_pose, base_pose_offset.pose, 
				pos_tolerance, rot_tolerance, 
				at_pos, at_rot,
				rotate
			)
			# rospy.logerr((at_pos, at_rot))

			# resp = self.move_head("LOOK_AT", dynamic_pose.ps)

			if self.command_state == self.CANCELLED :
				return
			
			self.pauseCommand() #TODO move fr
			if self.command_state == self.RUNNING :
				base_pose = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)

			base_pose_offset = getOffsetPose()
			rate.sleep()

	def grabObject(self, dynamic_pose) :

		holding_object = False
		while not holding_object :
			if self.command_state == self.CANCELLED :
				return
			self.pauseCommand()

			self.publishResponse("Attempting to grab %s_%s" % (dynamic_pose.color, dynamic_pose.id))
			resp = self.move_arm("OPEN_GRIPPER", PoseStamped())

			def getOffsetPose():
				# rospy.logerr(dynamic_pose.ps)
				base_pose = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)
				base_to_ar_t = transform_from_pose(base_pose.pose)

				offset_t = Transform()
				# offset_t.translation = Vector3(-0.093, -0.019, 0.005)
				offset_t.translation = Vector3(-0.023, 0.00, -0.025)
				offset_t.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
				# offset_t.rotation = Quaternion(0.620, 0.658, -0.305, -0.298)
				# offset_inv_t = inverse_transform(offset_t)

				base_to_offset_t = multiply_transforms(base_to_ar_t, offset_t)
				base_pose_offset = deepcopy(base_pose)
				base_pose_offset.pose = transform_to_pose(base_to_offset_t)

				# q = base_pose_offset.pose.orientation
				# rpy = euler_from_quaternion([q.x,q.y,q.z,q.w])
				# q = quaternion_from_euler(3.1415, 0.0, rpy[2])
				# base_pose_offset.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
				return base_pose_offset

			# base_pose_offset = self.transform_pose(self.BASE_FRAME, dynamic_pose.ps)
			base_pose_offset = getOffsetPose()
			resp = self.move_arm("MOVE_TO_POSE_INTERMEDIATE", base_pose_offset)

			if not resp.success:
				rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
				# try moving to it again
				self.servoBaseToDynamicPose(self.object_dp, .40, .08, yoff=.00)
				self.servoBaseToDynamicPose(self.object_dp, .25, .025, yoff=-.00)
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
			# base_pose.pose.orientation = Quaternion(-0.019, 0.741, 0.017, 0.671)
			self.test_pose_pub.publish(base_pose)
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

		#if not self.joystick_topic == "" : TODO
		self.servoBaseToDynamicPose(self.object_dp, .40, .08, yoff=.00, rotate=False)
		self.servoBaseToDynamicPose(self.object_dp, .25, .025, yoff=-.00)
		self.interActionDelay(1)

		self.grabObject(self.object_dp)
		self.interActionDelay(1)

		if self.target_dp.ps == None or self.target_dp.ps.pose.position == Point():
			self.search_sequence(self.target_dp)

		self.moveBaseToDynamicPos(self.target_dp)
		self.servoBaseToDynamicPose(self.target_dp, .25, .04, rotate=False)
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

		self.moveBaseToDynamicPos(self.target_dp)
		self.servoBaseToDynamicPose(self.target_dp, .30, .04)
		
		if self.command_state == self.RUNNING :
			self.publishResponse("finished moving base to target")
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished move base while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command")

	def search_sequence(self, dp) :

		rospy.logerr("Searching for %s_%s" % (dp.color, dp.id))
		
		# resp = self.move_head("LOOK_DOWN", PoseStamped())

		ps = PoseStamped()
		ps.header.frame_id = "base_link"
		ps.header.stamp = rospy.Time.now()
		ps.pose.position = Point(0.46, 0.00, 0.40)

		while dp.ps == None or dp.ps.pose.position == Point():
			ps.pose.position.z -= .05
			resp = self.move_head("LOOK_AT", ps)
			rospy.logerr("%s_%s not found yet" % (dp.color, dp.id))
			rospy.sleep(1)

		rospy.logerr("%s_%s found!" % (dp.color, dp.id))
		# resp = self.move_arm("RESET_ARM", PoseStamped())
		# if not resp.success:
		# 	rospy.logerr("RESET_ARM FAILED")

		# self.moveBaseToDynamicPos(self.target_dp)
		# self.servoBaseToDynamicPose(self.target_dp)
		
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

	def cancelgmapBaseTo(self) :
		self.gatlin_cmd_pub.publish(9)

	def gmapBaseTo(self, ps) :
		map_target_pose = self.transform_pose("map", ps)
		# map_robot_pose = self.transform_pose("map", self.robot_pose.ps)
		map_target_pose.pose.orientation = map_robot_pose.pose.orientation
		self.gmap_base_pub.publish(map_target_pose)

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
		# self.dm.add_ol_sub("/gatlin/ar_marker_list")
		self.dm.add_ol_sub("/gatlin/ar_marker_list/base_link")
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

		self.test_pose_pub = rospy.Publisher('/test_obj_pose2', PoseStamped)

		rospy.spin()

if __name__ == "__main__":
	Nav_Manip_Controller()