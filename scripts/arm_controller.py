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


class Arm_Controller:
	def __init__(self):
		# Give the launch a chance to catch up
		rospy.sleep(5)

		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		rospy.init_node('Arm_Controller')
		rospy.loginfo("Launched Arm Controller")

		# constants
		self.GROUP_NAME_ARM = 'arm'
		self.GRIPPER_FRAME = 'gripper_link'
		self.REFERENCE_FRAME = 'base_link'
		self.ARM_BASE_FRAME = 'arm_base_link'

		self.done = True

		self.test_pose_publisher = rospy.Publisher('/test_arm_pose', PoseStamped)

		rospy.Subscriber("/arm_target_pose", PoseStamped, self.move_arm_to_pose, queue_size=1)
		self.robot_name = "gatlin"
		move_arm_service = createService('move/arm', MoveRobot, self.handle_move_arm, self.robot_name)

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

	def MoveToPoseWithIntermediate(self, ps, offsets) :
		success = False
		for offset in offsets:
			# interpose = getOffsetPose(hand_pose, offset)
			interpose = getOffsetPose(ps, offset)
			success = self.MoveToPose(interpose, "MoveToIntermediatePose")

		success = self.MoveToPose(ps, "MoveToPose")

		return success

	def MoveToPose(self, ps, name) :
		newpose = self.transform_pose(self.REFERENCE_FRAME, ps)
		down = Quaternion(-0.00035087, 0.73273, 0.00030411, 0.68052)
		newpose.pose.orientation = down
		# newpose.position.z -= .03

		if self.move_arm_to_pose(newpose) :
			rospy.loginfo("SUCCEEDED: %s" % name)
			return True
		else :
			rospy.logerr("FAILED %s" % name)
			return False

	def move_arm_to_pose(self, ps):
		arm_target_pose = deepcopy(ps)
		arm_target_pose.header.stamp = rospy.Time.now()
		
		self.test_pose_publisher.publish(arm_target_pose)
		
		self.arm.set_pose_target(arm_target_pose)
		success = self.arm.go()

		return success

	def handle_move_arm(self, req):
		success = True
		gripper = self.gripper

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
			success = self.MoveToPoseWithIntermediate(req.ps, offsets)

		elif req.action == MOVE_TO_POSE :
			rospy.loginfo("Trying to Move To Pose")
			success = self.MoveToPose(req.ps, "FAILED MoveToPose")

		elif req.action == RESET_ARM :
			rospy.loginfo("Trying to Move To Rest Pose")
			success = self.move_arm_to_pose(self.rest_pose)
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

if __name__ == "__main__":
	Arm_Controller()