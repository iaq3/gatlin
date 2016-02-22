#!/usr/bin/env python

import rospy, sys, tf, moveit_commander
from math import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from gatlin.msg import *
from gatlin.srv import *
from config import *
from tf.transformations import *
from copy import deepcopy

class Gripper:
	def __init__(self):
		self.gripper_pub = rospy.Publisher('/gripper_joint/command', Float64)

	def set(self, val):
		self.gripper_pub.publish((1-val)*-2.53)
		rospy.sleep(2)

	def open(self, block=False):
		self.set(1.0)

	def close(self, block=False):
		self.set(0.3)


class Gatlin_Server:
	def __init__(self):
		# Give the launch a chance to catch up
		rospy.sleep(5)

		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		rospy.init_node('Gatlin_Server')
		print "Launched Gatlin Server"

		# constants
		self.GROUP_NAME_ARM = 'arm'
		self.GRIPPER_FRAME = 'gripper_link'
		self.REFERENCE_FRAME = 'base_link'
		self.ARM_BASE_FRAME = 'arm_base_link'

		self.done = True

		self.test_pose_publisher = rospy.Publisher('/test_arm_pose', PoseStamped)


		rospy.Subscriber("/arm_target_pose", Pose, self.move_arm_to_pose, queue_size=1)
		self.robot_name = "gatlin"
		move_arm_service = createService('move/arm', MoveRobot, self.handle_move_arm, self.robot_name)

		# rospy.Subscriber("/target_pos", Vector3, self.pos_callback, queue_size=1)
		rospy.Subscriber("/target_orientation", Vector3, self.orientation_cb, queue_size=1)

		# We need a tf listener to convert poses into arm reference base
		self.tfl = tf.TransformListener()

		# Initialize the move group for the right arm
		self.arm = MoveGroupCommander(self.GROUP_NAME_ARM)
		self.gripper = Gripper()

		self.robot = moveit_commander.RobotCommander()

		# Allow replanning to increase the odds of a solution
		self.arm.allow_replanning(True)

		# Set the planner
		self.arm.set_planner_id("RRTConnectkConfigDefault")

		# Set the right arm reference frame
		self.arm.set_pose_reference_frame(self.REFERENCE_FRAME)

		# Give the scene a chance to catch up
		rospy.sleep(1)

		# Allow some leeway in position (meters) and orientation (radians)
		# USELESS; do not work on pick and place! Explained on this issue:
		# https://github.com/ros-planning/moveit_ros/issues/577
		self.arm.set_goal_position_tolerance(0.005)
		self.arm.set_goal_orientation_tolerance(0.05)

		# Allow 2 seconds per planning attempt
		self.arm.set_planning_time(2.0)

		# Create a quaternion from the Euler angles
		#q = quaternion_from_euler(0, pitch, yaw)
		# horiz = Quaternion(-0.023604, 0.99942, 0.00049317, 0.024555)
		# deg45 = Quaternion(-0.022174, 0.9476, 0.0074215, 0.31861)
		down = Quaternion(-0.00035087, 0.73273, 0.00030411, 0.68052)
		# back_pos = Point(-0.03, 0.0313, 0.476)

		# init rest pose
		self.rest_pose = PoseStamped()
		self.rest_pose.header.frame_id = self.REFERENCE_FRAME
		self.rest_pose.pose.position = Point(-0.0762700684061, 0.0105568131039, 0.290791199591)
		self.rest_pose.pose.orientation = Quaternion(0.0251355325061, 0.982948881414, -0.0046583987932, 0.182093384981)

		# init current pose
		self.current_pose = PoseStamped()
		self.current_pose.header.frame_id = self.REFERENCE_FRAME
		self.current_pose.pose.position = Point(0,0,0)
		self.current_pose.pose.orientation = down

		# Open the gripper
		rospy.loginfo("Set Gripper: open")
		self.gripper.set(1.0)

		# self.arm.set_pose_target(self.rest_pose)
		# self.arm.go()
		# rospy.sleep(1)

		rospy.spin()

	def MoveToPoseWithIntermediate(self, pose, offsets) :
		# arm = self.limb_left if limb == 'left' else self.limb_right
		# hand_pose = self.getCurrentPose(arm)
		success = False
		for offset in offsets:
			# interpose = getOffsetPose(hand_pose, offset)
			interpose = getOffsetPose(pose, offset)
			success = self.MoveToPose(interpose, "MoveToIntermediatePose")

		success = self.MoveToPose(pose, "MoveToPose")

		return success

	def MoveToPose(self, pose, name) :
		newpose = deepcopy(pose)
		down = Quaternion(-0.00035087, 0.73273, 0.00030411, 0.68052)
		newpose.orientation = down
		# newpose.position.z -= .03

		if self.move_arm_to_pose(newpose) :
			rospy.loginfo("SUCCEEDED: %s" % name)
			return True
		else :
			rospy.logerr("FAILED %s" % name)
			return False

	def move_arm_to_pose(self, pose):
		arm_target_pose = PoseStamped()
		arm_target_pose.header.frame_id = self.REFERENCE_FRAME
		arm_target_pose.header.stamp = rospy.Time.now()
		arm_target_pose.pose = deepcopy(pose)
		
		self.test_pose_publisher.publish(arm_target_pose)
		# rospy.logerr(arm_target_pose)
		
		self.arm.set_pose_target(arm_target_pose)
		success = self.arm.go()

		return success

	def handle_move_arm(self, req):
		# rospy.sleep(1)
		success = True
		gripper = self.gripper

		# if not (req.limb == 'left' or req.limb == 'right'):
			# rospy.logerr("No Limb Set")

		if req.action == OPEN_GRIPPER:
			rospy.loginfo("Beginning to open gripper")
			gripper.open(block=True)
			rospy.loginfo("Opened Gripper")

		elif req.action == CLOSE_GRIPPER :
			rospy.loginfo("Beginning to close Gripper")
			gripper.close(block=True)
			rospy.loginfo("Closed Gripper")

		elif req.action == MOVE_TO_POSE_INTERMEDIATE :
			rospy.loginfo("Trying to Move To Pose With Intermediate")
			offsets = [Vector3(0,0,.07)]
			success = self.MoveToPoseWithIntermediate(req.pose, offsets)

		elif req.action == MOVE_TO_POSE :
			rospy.loginfo("Trying to Move To Pose")
			success = self.MoveToPose(req.pose, "FAILED MoveToPose")

		elif req.action == RESET_ARM :
			rospy.loginfo("Trying to Move To Rest Pose")
			success = self.move_arm_to_pose(self.rest_pose.pose)
			# success = self.MoveToPose(self.rest_pose, "FAILED MoveToRestPose")

		# elif req.action == MOVE_TO_POS :
		# 	rospy.loginfo("Trying to Move To Pos")

		# 	new_pose = Pose()
		# 	if req.limb == 'left':
		# 		try:
		# 			self.initial_left
		# 			new_pose = deepcopy(self.initial_left)
		# 		except AttributeError:
		# 			new_pose = deepcopy(self.hand_pose_left)
		# 			self.initial_left = deepcopy(self.hand_pose_left)
		# 	elif req.limb == 'right':
		# 		try:
		# 			self.initial_right
		# 			new_pose = deepcopy(self.initial_right)
		# 		except AttributeError:
		# 			new_pose = deepcopy(self.hand_pose_right)
		# 			self.initial_right = deepcopy(self.hand_pose_right)

		# 	new_pose.position = deepcopy(req.pose.position)
		# 	# success = self.MoveToPose(req.limb, new_pose, "FAILED MoveToPose")
		# 	success = self.MoveToPoseWithIntermediate(req.limb, new_pose)
		# 	rospy.loginfo("Moved to pos: %r" % success)

		else :
			success = False
			rospy.logerr("invalid action")

		return MoveRobotResponse(success)


	# #takes a point from kinect to base frame
	# def kinect_to_base(self, data) :
	# 	rospy.loginfo("kinect to base!: [%f, %f, %f]"%(data.x, data.y, data.z))
		
	# 	target_pos = [data.x, -data.y, data.z]

	# 	kinectPt = PointStamped()
	# 	kinectPt.header.frame_id = "/camera_rgb_optical_frame"
		
	# 	# kinectPt.header.stamp = rospy.Time.now() - rospy.Duration(.22)
	# 	kinectPt.header.stamp = rospy.Time(0)
	# 	kinectPt.point = Point(target_pos[0],target_pos[1],target_pos[2])

	# 	basePt = self.tfl.transformPoint(self.REFERENCE_FRAME, kinectPt)

	# 	print "############# basePt #############"
	# 	print basePt
	# 	return basePt.point

	def orientation_cb(self, data):
		if(data.x == -1.0 and data.y == -2.0):
			print "################################################"
			print "###################  Orientation!  #############"
			print "################################################"
			deg = data.z*15
			rad = -deg * pi/180 + pi/2
			print rad
			q = quaternion_from_euler(0,rad,0)
			self.current_pose.pose.orientation.x = q[0]
			self.current_pose.pose.orientation.y = q[1]
			self.current_pose.pose.orientation.z = q[2]
			self.current_pose.pose.orientation.w = q[3]
		else:
			return

	# def pos_callback(self, data):
	# 	if self.done:
	# 		if(data.x == 0.0 and data.y == 0.0 and data.z == 0.0):
	# 			self.done = False
	# 			print "################################################"
	# 			print "###################  Going!  ###################"
	# 			print "################################################"
	# 			if self.arm.go() != True:
	# 				rospy.logwarn("Go to target_pos failed")
	# 			# rospy.sleep(.2)
	# 			self.done = True

	# 		elif(data.x == -1.0 and data.y == -1.0):
	# 			return		
	# 		elif(data.x == -1.0 and data.y == -2.0):
	# 			return
	# 		elif(data.x == -2.0 and data.y == -2.0 and data.z == -2.0):
	# 			print "################################################"
	# 			print "###################  Reset!  ###################"
	# 			print "################################################"

	# 			self.arm.set_pose_target(self.rest_pose)
	# 			self.arm.go()

	# 			# rospy.loginfo("Set Gripper: open")
	# 			#self.gripper.set(1.0)

	# 			rospy.sleep(1)
	# 			return		
	# 		else:

	# 			rospy.loginfo("Received a target_pos message: [%f, %f, %f]"%(data.x, data.y, data.z))

	# 			target_pos = [data.x, -data.y, data.z]

	# 			kinectPt = PointStamped()
	# 			kinectPt.header.frame_id = "/camera_rgb_optical_frame"
	# 			# kinectPt.header.stamp = rospy.Time.now() - rospy.Duration(.22)
	# 			kinectPt.header.stamp = rospy.Time(0)
	# 			kinectPt.point = Point(target_pos[0],target_pos[1],target_pos[2])

	# 			basePt = self.tfl.transformPoint("/base_link", kinectPt)

	# 			print "############# basePt #############"
	# 			print basePt

	# 			# Set a target pose for the arm        
	# 			target_pose = PoseStamped()
	# 			target_pose = self.current_pose

	# 			target_pose.pose.position = basePt.point

	# 			# horiz = Quaternion(-1.56720103577e-05, -0.00267706753017, -0.00511322338149, 0.999983343867)
	# 			q = quaternion_from_euler(0,0,0)
	# 			horiz = Quaternion(q[0],q[1],q[2],q[3])

	# 			# deg45 = Quaternion(-0.00229553611512, 0.448584123035, 0.00456900568013, 0.893725986677)
	# 			q = quaternion_from_euler(0,.785,0)
	# 			deg45 = Quaternion(q[0],q[1],q[2],q[3])

	# 			# down = Quaternion(-0.00366978827416, 0.71742069389, 0.00356061132163, 0.696621419911)
	# 			q = quaternion_from_euler(0,1.57,0)
	# 			down = Quaternion(q[0],q[1],q[2],q[3])

	# 			# target_pose.pose.orientation = deg45
	# 			target_pose.pose.orientation = self.current_pose.pose.orientation

	# 			print "############# current_pose #############"
	# 			print self.arm.get_current_pose()

	# 			target_pose.pose.position.z -= .03
	# 			# target_pose.pose.position.y += .01

	# 			inter_pose = deepcopy(target_pose)
	# 			inter_pose.pose.position.z += .05

	# 			self.arm.set_pose_target(inter_pose)
	# 			self.arm.go()

	# 			self.arm.set_pose_target(target_pose)
	# 			self.arm.go()
	# 			rospy.sleep(.05)

if __name__ == "__main__":
	Gatlin_Server()