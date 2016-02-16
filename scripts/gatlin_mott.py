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
# from turtle_sim.msg import Velocity
from config import *
from gatlin.msg import *
	

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

class Mott_Thread(Thread) :
	def __init__(self, gm):
		Thread.__init__(self)
		self.gatlin_mott = gm
		self.lock = Lock()
		self.base_joystick_pub = rospy.Publisher("/cmd_vel_mux/input/teleop", Twist)

		# self.test_pose_pub = rospy.Publisher('/test_obj_pose', PoseStamped)


		#todo add lock

	def update_info(self, on, tn) :
		self.object_name = on
		self.target_name = tn

	#find the pose of object-pose in child transform when it is a child of parent transform
	def get_pose(self, parent, child, atom):
		try:
			ps = PoseStamped()
			ps.pose.orientation = atom.orientation
			ps.pose.position = atom.position
			ps.header.frame_id = child
			ps.header.stamp =  rospy.Time(0)
			self.tfl.waitForTransform(child, parent, rospy.Time(0), rospy.Duration(4.0))
			child_pose = self.tfl.transformPose(parent, ps)
			return child_pose
		except Exception as e:
			print e
			print "no transform"
			return None

	def servo_base_to_pose(self, obj_position) :
		actual_pos = vector3_to_numpy(obj_position)#self.gatlin_mott.object_pose.position
		actual_pos[2] = 0

		desired_pos = np.array([.29,0,0])

		error_vec =  actual_pos - desired_pos
		error = np.linalg.norm(error_vec)
		rospy.logerr(error_vec)

		forward = error_vec[0]
		turn = error_vec[1]

		error_angle = angle(error_vec, np.array([1,0,0]))
		rospy.logerr(error_angle)
		if error_angle > 0.174533 :
			forward = 0

		maxVel = .05
		mag = (turn**2 + forward**2)**.5
		if (mag > maxVel) :
			turn = (turn/mag) * maxVel
			forward = (forward/mag) * maxVel

		turn *= 3.0

		msg = Twist (Point(forward, 0.0, 0.0), Point(0.0, 0.0, turn))
		# rospy.logerr(msg)
		self.base_joystick_pub.publish(msg)

		return error


	def moveBaseToObject(self) :
		#gmap move base to object ******************************* TODO if not in visible frame, then 
		if self.gatlin_mott.distanceToObject() > 1.5 :#these are new
			self.gatlin_mott.publishResponse("Gmap base to "+self.object_name)
			
			#object pose is in kinect coordinates.... need them in map coordinates..... TODO test
			object_in_map = None
			while not object_in_map :
				object_in_map = self.get_pose('map', 'camera_link', self.gatlin_mott.object_pose)
			self.gatlin_mott.gmapBaseTo(object_in_map)

			#distance is from kinect...
			while self.gatlin_mott.distanceToObject() > 3 :
				time.sleep(.03)
			#stop gmap base
			self.gatlin_mott.cancelgmapBaseTo()

	def servoBaseToObject(self) :
		#servo base to object ************************************
		if self.gatlin_mott.distanceToObject() < 1.5 :
			self.gatlin_mott.publishResponse("Servo base to "+self.object_name)

			rate = rospy.Rate(30)
			error = self.gatlin_mott.distanceToObject()
			goal_tolerence = .05

			actual_pos = vector3_to_numpy(self.gatlin_mott.object_pose.position)
			actual_pos[2] = 0
			desired_pos = np.array([.27,0,0])
			error_vec =  actual_pos - desired_pos
			error = np.linalg.norm(error_vec)

			while error > goal_tolerence :
				error = self.servo_base_to_pose(self.gatlin_mott.object_pose.position)#self.gatlin_mott.object_pose.position
				rate.sleep()

			time.sleep(1)

	def grabObject(self) :
		#grab object **********************************************
		holding_object = False
		while not holding_object :
			print "attempting to grab"
			self.gatlin_mott.publishResponse("Attempting to grab "+self.object_name)
			#open gripper
			self.gatlin_mott.sendGripCommand(1)

			#arm to object
			print "sending arm pose pub"
			print self.gatlin_mott.object_pose
			self.gatlin_mott.arm_pose_pub.publish(self.gatlin_mott.object_pose)

			time.sleep(7)

			#grab
			print "grabbing arm"
			self.gatlin_mott.sendGripCommand(.5)
			time.sleep(3)

			#arm up and
			self.gatlin_mott.sendResetArm()

			time.sleep(7)

			if time.time() - self.gatlin_mott.last_object_pose_update > 1 : #no object detection in last second, it is likely in robot's hand
				holding_object = True

		self.gatlin_mott.publishResponse("Grabbed "+self.object_name)

	def moveBaseToTarget(self) :
		#gmap move base to target
		if self.gatlin_mott.distanceToTarget() > .7 :
			self.gatlin_mott.publishResponse("Gmap base to "+self.target_name)
			self.gatlin_mott.gmapBaseTo(self.gatlin_mott.target_pose)
			while self.gatlin_mott.distanceToTarget() > .6 :
				time.sleep(.03)
			#stop gmap base
			self.gatlin_mott.cancelgmapBaseTo()

	def servoBaseToTarget(self) :
		#if self.gatlin_mott.distanceToTarget() < 1.5 : #TODO these can be made variables these spots are where changes need to be made
		self.gatlin_mott.publishResponse("Servo base to "+self.target_name) #TODO

		rate = rospy.Rate(30)
		error = self.gatlin_mott.distanceToTarget() #TODO
		goal_tolerence = .05

		actual_pos = vector3_to_numpy(self.gatlin_mott.target_pose.position) #TODO
		actual_pos[2] = 0
		desired_pos = np.array([.27,0,0])
		error_vec =  actual_pos - desired_pos
		error = np.linalg.norm(error_vec)

		while error > goal_tolerence :
			error = self.servo_base_to_pose(self.gatlin_mott.target_pose.position)#TODO
			rate.sleep()

		time.sleep(1)

	def moveArmToTarget(self) :
		#move arm to target
		self.gatlin_mott.publishResponse("Moving Arm to "+self.target_name)
		self.gatlin_matt.arm_pose_pub.publish(self.gatlin_mott.target_pose)
		

		time.sleep(7)

		#release
		self.gatlin_mott.sendGripCommand(1)

		time.sleep(2)

		self.gatlin_mott.sendResetArm()

	def run_mott_sequence(self) :
		print "about to start with lock"
		with self.lock :
			print "inside lock"
			#if self.quitting :
			#	return
			#self.moveBaseToObject()
			self.servoBaseToObject()
			self.grabObject()

			# #self.moveBaseToTarget() TODO ready to test
			self.servoBaseToTarget()
			self.moveArmToTarget()

			self.gatlin_mott.publishResponse("finished")
			


	def run(self) :
		while not rospy.is_shutdown() :
			time.sleep(1)

		


class gatlin_mott:

	def MottCallback(self, data) :
		with self.mott_callback_lock : #guarantees sequential callbacks
			if not self.mott_thread.lock.locked() : #TODO might be bad practice, but only this method calls the thread
				print "callback of unlocked node"
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
				
				self.mott_thread.update_info(data.object_pose_topic, data.target_pose_topic) #maybe put this above?? lol
				self.mott_thread.run_mott_sequence()
			else :
				print "MOTT THREAD IS LOCKED!!!"

	def robotPoseCallback(self, data) :
		self.robot_pose = data

	def objectPoseCallback(self, data) :
		self.last_object_pose_update = time.time()
		self.object_pose = data

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

	def __init__(self):
		rospy.init_node('gatlin_mott')

		self.robot_pose = Pose()
		self.object_pose = Pose()
		self.last_object_pose_update = 0
		self.target_pose = Pose()

		self.mott_thread = Mott_Thread(self)
		self.mott_callback_lock = Lock()

		self.response_pub = rospy.Publisher("/gatlin_mott_response", String)
		self.gmap_base_pub = rospy.Publisher("/move_to_goal", Pose)
		self.gripper_pub = rospy.Publisher("/target_pos", Vector3)
		self.gatlin_cmd_pub = rospy.Publisher("/gatlin_cmd", Int32)
		self.arm_pose_pub = rospy.Publisher("/arm_target_pose", Pose)
		self.base_joystick_pub = rospy.Publisher("/cmd_vel_mux/input/teleop" , Twist)


		rospy.Subscriber("/gatlin_mott", Mott, self.MottCallback, queue_size = 1)
		rospy.Subscriber("/robot_pose", Pose, self.robotPoseCallback, queue_size =1)


		self.object_sub = rospy.Subscriber("/green_kinect0_pose", Pose, self.objectPoseCallback, queue_size = 1)
		self.target_sub = rospy.Subscriber("/green_kinect0_pose", Pose, self.targetPoseCallback, queue_size = 1)

		rospy.spin()


if __name__ == "__main__":
	gatlin_mott()