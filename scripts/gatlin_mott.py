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
	

def PointDistance (p1, p2) :
	return ((p1.x-p2.x)** 2 + (p1.y - p2.y)** 2 + (p1.z - p2.z)**2)**.5

def PointDistanceFlat (p1, p2) :
	return ((p1.x-p2.x)** 2 + (p1.y - p2.y)** 2)**.5

def PointMinus(p1, p2) :
	np = Point()
	np = p1.x - p2.x
	np = p1.y - p2.y
	np = p1.z - p2.z
	return np

def length(v):
  return math.sqrt(np.dot(v, v))

def angle(v1, v2):
  return math.acos(np.dot(v1, v2) / (length(v1) * length(v2)))

def relativeAngle(robot_pose, toTargetPoint) :
	toTarget = vector3_to_numpy(toTargetPoint)
	forwardRobot = np.array([0,-1,0, 1])
	robotRotation = quaternion_matrix(robot_pose.orientation)
	forwardRobotInMap = np.dot(robotRotation, forwardRobot.T)
	forwardRobotInMap = forwardRobotInMap[:3]/forwardRobotInMap[3]
	print "forward robot in map"
	print forwardRobotInMap
	return angle(forwardRobotInMap, toTarget)

class Nav_Manip_Controller :

	def update_info(self, on, tn) :
		self.object_name = on
		self.target_name = tn

	def servo_base_to_pose(self, desired_pos, obj_position) :
		actual_pos = vector3_to_numpy(obj_position)#self.object_pose.position
		actual_pos[2] = 0

		# rospy.logerr(obj_position)

		error_vec =  actual_pos - desired_pos
		error = np.linalg.norm(error_vec)
		# rospy.logerr(error_vec)

		forward = error_vec[0]
		turn = error_vec[1]

		error_angle = angle(error_vec, np.array([1,0,0]))
		# rospy.logerr(error_angle)
		if error_angle > 0.25 :
			forward = 0

		maxVel = .08
		minVel = .04
		mag = (turn**2 + forward**2)**.5
		if (mag > maxVel) :
			turn = (turn/mag) * maxVel
			forward = (forward/mag) * maxVel

		if (mag < minVel) :
			turn = (turn/mag) * minVel
			forward = (forward/mag) * minVel

		turn *= 1.5

		msg = Twist (Point(forward, 0.0, 0.0), Point(0.0, 0.0, turn))
		# rospy.logerr(msg)
		self.base_joystick_pub.publish(msg)

		return error


	def moveBaseToObject(self) :
		#gmap move base to object ******************************* TODO if not in visible frame, then 
		if self.distanceToObject() > .7 :#these are new
			self.publishResponse("Gmap base to "+self.object_name)
			
			#object pose is in kinect coordinates.... need them in map coordinates..... TODO test
			#object_in_map = None
			#while not object_in_map :
			#	object_in_map = self.transform_pose('map', 'camera_link', self.object_pose)
			map_obj_pose = self.transform_pose("map", self.FIXED_FRAME, self.object_pose)
			# todo: check_transform_pose("map", self.object_pose_stamped)
			self.gmapBaseTo(map_obj_pose)

			#distance is from kinect...
			while self.distanceToObject() > .7 :
				time.sleep(.03)
			#stop gmap base
			self.cancelgmapBaseTo()

	def servoBaseToObject(self) :
		#servo base to object ************************************
		#if self.distanceToObject() < 1.5 :
		self.publishResponse("Servo base to "+self.object_name)

		rate = rospy.Rate(30)
		error = self.distanceToObject()
		goal_tolerence = .02

		base_obj_pose = self.transform_pose("base_link", self.FIXED_FRAME, self.object_pose)

		actual_pos = vector3_to_numpy(base_obj_pose.pose.position)
		actual_pos[2] = 0
		desired_pos = np.array([.25,0,0])
		error_vec =  actual_pos - desired_pos
		error = np.linalg.norm(error_vec)

		while error > goal_tolerence :
			base_obj_pose = self.transform_pose("base_link", self.FIXED_FRAME, self.object_pose)
			error = self.servo_base_to_pose(desired_pos, base_obj_pose.pose.position)
			rate.sleep()

		time.sleep(1)

	def grabObject(self) :
		#grab object **********************************************
		holding_object = False
		while not holding_object :
			print "attempting to grab"
			self.publishResponse("Attempting to grab "+self.object_name)
			#open gripper
			resp = self.move_arm(OPEN_GRIPPER, Pose())

			#arm to object
			print "sending arm pose pub"
			print self.object_pose
			# self.arm_pose_pub.publish(self.object_pose)
			base_obj_pose = self.transform_pose("base_link", self.FIXED_FRAME, self.object_pose)
			resp = self.move_arm(MOVE_TO_POSE_INTERMEDIATE, base_obj_pose.pose)
			if not resp.success:
				rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")

			#grab
			print "grabbing arm"
			# self.sendGripCommand(.5)
			resp = self.move_arm(CLOSE_GRIPPER, Pose())

			#arm up and
			# self.sendResetArm()
			resp = self.move_arm(RESET_ARM, Pose())
			if not resp.success:
				rospy.logerr("RESET_ARM FAILED")

			if time.time() - self.last_object_pose_update > 1 : #no object detection in last second, it is likely in robot's hand
				holding_object = True

		self.publishResponse("Grabbed "+self.object_name)

	def moveBaseToTarget(self) :
		#gmap move base to target
		if self.distanceToTarget() > .7 :
			self.publishResponse("Gmap base to "+self.target_name)
			self.gmapBaseTo(self.target_pose)
			while self.distanceToTarget() > .7 :
				time.sleep(.03)
			#stop gmap base
			self.cancelgmapBaseTo()

	def servoBaseToTarget(self) :
		#if self.distanceToTarget() < 1.5 : #TODO these can be made variables these spots are where changes need to be made
		self.publishResponse("Servo base to "+self.target_name) #TODO

		rate = rospy.Rate(30)
		error = self.distanceToTarget() #TODO
		goal_tolerence = .05

		base_target_pose = self.transform_pose("base_link", self.FIXED_FRAME, self.target_pose)

		actual_pos = vector3_to_numpy(base_target_pose.pose.position) #TODO
		actual_pos[2] = 0
		desired_pos = np.array([.25,0,0])
		error_vec =  actual_pos - desired_pos
		error = np.linalg.norm(error_vec)

		while error > goal_tolerence : #might be bad idea to wait for transform in visual servo system....
			base_target_pose = self.transform_pose("base_link", self.FIXED_FRAME, self.target_pose)

			self.test_pose_pub.publish(base_target_pose)

			error = self.servo_base_to_pose(desired_pos, base_target_pose.pose.position)#TODO
			rate.sleep()

		time.sleep(1)

	def moveArmToTarget(self) :
		#move arm to target
		self.publishResponse("Moving Arm to "+self.target_name)
		#self.gatlin_matt.arm_pose_pub.publish(self.target_pose)
		base_target_pose = self.transform_pose("base_link", self.FIXED_FRAME, self.target_pose)
		resp = self.move_arm(MOVE_TO_POSE_INTERMEDIATE, base_target_pose.pose)

		#time.sleep(1)

		#release
		self.sendGripCommand(1)

		time.sleep(2)

		self.sendResetArm()
		#resp = self.move_arm(RESET_ARM, Pose())

	def run_mott_sequence(self) :
		# print "about to start with lock"
		# with self.lock :
		# 	print "inside lock"
			#if self.quitting :
			#	return
		self.moveBaseToObject()
		self.servoBaseToObject()
		self.grabObject()

		self.moveBaseToTarget() #TODO ready to test
		self.servoBaseToTarget()
		self.moveArmToTarget()

		self.publishResponse("finished mott") #string must contain finished

	def base_to_sequence(self) :
		
		self.moveBaseToTarget()
		self.servoBaseToTarget()
		self.publishResponse("finished moving base to target")

		
	def MottCallback(self, data) :
		# with self.mott_callback_lock : #guarantees sequential callbacks
			# if not self.lock.locked() : #TODO might be bad practice, but only this method calls the thread
		# print "callback of unlocked node"
		self.object_sub.unregister()
		self.target_sub.unregister()

		if (not (data.object_pose_topic == "")) :
			self.object_sub = rospy.Subscriber(data.object_pose_topic, Pose, self.objectPoseCallback, queue_size = 1)
		if (not (data.target_pose_topic == "")) :
			self.target_sub = rospy.Subscriber(data.target_pose_topic, Pose, self.targetPoseCallback, queue_size = 1)

		if (data.object_pose) :
			self.object_pose = data.object_pose
		if (data.target_pose) :
			self.target_pose = data.target_pose
		
		self.update_info(data.object_pose_topic, data.target_pose_topic) #maybe put this above?? lol
		
		if data.command == "mott" :
			self.run_mott_sequence()
		elif data.command == "move_base" :
			print "Starting Move Base TO"
			self.base_to_sequence()
			# else :
			# 	print "MOTT THREAD IS LOCKED!!!"

	def robotPoseCallback(self, data) :
		self.robot_pose = data

	def objectPoseCallback(self, data) :
		fixed_obj_pose = self.transform_pose(self.FIXED_FRAME, "camera_rgb_optical_frame", data)
		self.last_object_pose_update = time.time()
		self.object_pose = fixed_obj_pose.pose

	def targetPoseCallback(self, data) :
		self.target_pose = data

	def baseJoystickPublish (msg) :
		self.base_joystick_pub.publish(msg)

	def cancelgmapBaseTo(self) :
		self.gatlin_cmd_pub.publish(9)

	def sendGripCommand(self, val) :
		msg = Vector3()
		msg.x = -1
		msg.y = -1
		msg.z = val
		self.gripper_pub.publish(msg)

	def sendResetArm(self) :
		msg = Vector3()
		msg.x = -2
		msg.y = -2
		msg.z = -2
		self.gripper_pub.publish(msg)

	def gmapBaseTo(self, p) :
		p.orientation = self.robot_pose.orientation
		self.gmap_base_pub.publish(p)

	def publishResponse(self, statement) : 
		self.response_pub.publish(statement)

	#object is in kinect frame. find distance to origin ie magnitude
	def distanceToObject(self) :
		z = Point()
		z.x = 0
		z.y = 0
		z.z = 0
		return PointDistance(z, self.object_pose.position)

	#distance from robot_pose to target_pose in map frame
	def distanceToTarget(self) :
		return PointDistance(self.robot_pose.position, self.target_pose.position)

	#find the pose of object-pose in child transform when it is a child of parent transform
	def get_pose(self, parent, ps):
		try:
			ps = deepcopy(ps)
			ps.header.stamp =  rospy.Time(0)
			self.tfl.waitForTransform(ps.header.frame_id, parent, rospy.Time(0), rospy.Duration(4.0))
			child_pose = self.tfl.transformPose(parent, ps)
			return child_pose
		except Exception as e:
			print e
			print "no transform"
			return None

	def __init__(self):
		self.robot_name = "gatlin"
		rospy.init_node('%s_nav_manip_controller'%self.robot_name)

		self.robot_pose = Pose()
		self.object_pose = Pose()
		self.last_object_pose_update = 0
		self.target_pose = Pose()

		# self = Nav_Manip_Controller()
		# self.mott_callback_lock = Lock()
		self.base_joystick_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist)
		self.test_pose_pub = rospy.Publisher('/test_obj_pose', PoseStamped)

		self.FIXED_FRAME = "odom"

		self.response_pub = rospy.Publisher("/gatlin_mott_response", String)
		self.gmap_base_pub = rospy.Publisher("/move_to_goal", Pose)
		self.gripper_pub = rospy.Publisher("/target_pos", Vector3)
		self.gatlin_cmd_pub = rospy.Publisher("/gatlin_cmd", Int32)
		self.base_joystick_pub = rospy.Publisher("/cmd_vel_mux/input/teleop" , Twist)

		self.arm_pose_pub = rospy.Publisher("/arm_target_pose", Pose)

		rospy.Subscriber("/gatlin_mott", Mott, self.MottCallback, queue_size = 1)
		rospy.Subscriber("/robot_pose", Pose, self.robotPoseCallback, queue_size =1)

		self.move_arm = createServiceProxy("move/arm", MoveRobot, self.robot_name)

		self.object_sub = rospy.Subscriber("/green_kinect0_pose", Pose, self.objectPoseCallback, queue_size = 1)
		self.target_sub = rospy.Subscriber("/green_kinect0_pose", Pose, self.targetPoseCallback, queue_size = 1)

		self.tfl = tf.TransformListener()

		rospy.spin()

class DynamicPose:
	def __init__(self):
		self.subscribed = False
		self.topic = ""
		self.ps = PoseStamped()

	def subscribe(topic):
		self.object_sub = rospy.Subscriber(topic, Pose, self.set_pose, queue_size = 1)

	def set_pose(ps):
		self.ps = deepcopy(ps)


if __name__ == "__main__":
	Nav_Manip_Controller()