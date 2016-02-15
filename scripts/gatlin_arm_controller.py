#!/usr/bin/env python

import rospy, sys, tf, moveit_commander
from math import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import *
from moveit_msgs.srv import *
from tf.transformations import *
from copy import deepcopy

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

		# init a gripper publisher because movegroup won't work
		self.gripper_pub = rospy.Publisher('/gripper_joint/command', Float64)

		self.test_pose_publisher = rospy.Publisher('/test_arm_pose3', PoseStamped)


		rospy.Subscriber("/arm_target_pose", Pose, self.arm_target_pose_cb, queue_size=1)
		rospy.Subscriber("/target_pos", Vector3, self.pos_callback, queue_size=1)
		rospy.Subscriber("/target_orientation", Vector3, self.orientation_cb, queue_size=1)

		# We need a tf listener to convert poses into arm reference base
		self.tfl = tf.TransformListener()

		# Initialize the move group for the right arm
		self.arm = MoveGroupCommander(self.GROUP_NAME_ARM)

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
		self.arm.set_goal_position_tolerance(0.01)
		self.arm.set_goal_orientation_tolerance(0.2)

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
		self.gripper_set(1.0)

		self.arm.set_pose_target(self.rest_pose)
		# self.arm.go()
		rospy.sleep(1)

		rospy.spin()


	def arm_target_pose_cb(self, msg):
		# base_rel_pose = self.kinect_to_base(msg.position)

		kinectPt = PointStamped()
		kinectPt.header.frame_id = "/camera_rgb_optical_frame"
		kinectPt.header.stamp = rospy.Time(0)
		kinectPt.point = deepcopy(msg.position)

		basePt = self.tfl.transformPoint(self.REFERENCE_FRAME, kinectPt)

		arm_target_pose = PoseStamped()
		arm_target_pose.header.frame_id = self.REFERENCE_FRAME
		arm_target_pose.header.stamp = rospy.Time.now()
		arm_target_pose.pose.position = deepcopy(basePt.point)
		down = Quaternion(-0.00035087, 0.73273, 0.00030411, 0.68052)
		arm_target_pose.pose.orientation = down
		arm_target_pose.pose.position.z += .015
		self.test_pose_publisher.publish(arm_target_pose)
		rospy.logerr(arm_target_pose)

		inter_pose = deepcopy(arm_target_pose)
		inter_pose.pose.position.z += .05

		self.arm.set_pose_target(inter_pose)
		self.arm.go()
		
		self.arm.set_pose_target(arm_target_pose)
		self.arm.go()

		rospy.sleep(1)

	def gripper_set(self, val):
		self.gripper_pub.publish((1-val)*-2.53)

	#takes a point from kinect to base frame
	def kinect_to_base(self, data) :
		rospy.loginfo("kinect to base!: [%f, %f, %f]"%(data.x, data.y, data.z))
		
		target_pos = [data.x, -data.y, data.z]

		kinectPt = PointStamped()
		kinectPt.header.frame_id = "/camera_rgb_optical_frame"
		
		# kinectPt.header.stamp = rospy.Time.now() - rospy.Duration(.22)
		kinectPt.header.stamp = rospy.Time(0)
		kinectPt.point = Point(target_pos[0],target_pos[1],target_pos[2])

		basePt = self.tfl.transformPoint(self.REFERENCE_FRAME, kinectPt)

		print "############# basePt #############"
		print basePt
		return basePt.point

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

	def pos_callback(self, data):
		if self.done:
			if(data.x == 0.0 and data.y == 0.0 and data.z == 0.0):
				self.done = False
				print "################################################"
				print "###################  Going!  ###################"
				print "################################################"
				if self.arm.go() != True:
					rospy.logwarn("Go to target_pos failed")
				# rospy.sleep(.2)
				self.done = True

			elif(data.x == -1.0 and data.y == -1.0):
				return		
			elif(data.x == -1.0 and data.y == -2.0):
				return
			elif(data.x == -2.0 and data.y == -2.0 and data.z == -2.0):
				print "################################################"
				print "###################  Reset!  ###################"
				print "################################################"

				self.arm.set_pose_target(self.rest_pose)
				self.arm.go()

				# rospy.loginfo("Set Gripper: open")
				#self.gripper_set(1.0)

				rospy.sleep(1)
				return		
			else:

				rospy.loginfo("Received a target_pos message: [%f, %f, %f]"%(data.x, data.y, data.z))

				target_pos = [data.x, -data.y, data.z]

				kinectPt = PointStamped()
				kinectPt.header.frame_id = "/camera_rgb_optical_frame"
				# kinectPt.header.stamp = rospy.Time.now() - rospy.Duration(.22)
				kinectPt.header.stamp = rospy.Time(0)
				kinectPt.point = Point(target_pos[0],target_pos[1],target_pos[2])

				basePt = self.tfl.transformPoint("/base_link", kinectPt)

				print "############# basePt #############"
				print basePt

				# Set a target pose for the arm        
				target_pose = PoseStamped()
				target_pose = self.current_pose

				target_pose.pose.position = basePt.point

				# horiz = Quaternion(-1.56720103577e-05, -0.00267706753017, -0.00511322338149, 0.999983343867)
				q = quaternion_from_euler(0,0,0)
				horiz = Quaternion(q[0],q[1],q[2],q[3])

				# deg45 = Quaternion(-0.00229553611512, 0.448584123035, 0.00456900568013, 0.893725986677)
				q = quaternion_from_euler(0,.785,0)
				deg45 = Quaternion(q[0],q[1],q[2],q[3])

				# down = Quaternion(-0.00366978827416, 0.71742069389, 0.00356061132163, 0.696621419911)
				q = quaternion_from_euler(0,1.57,0)
				down = Quaternion(q[0],q[1],q[2],q[3])

				# target_pose.pose.orientation = deg45
				target_pose.pose.orientation = self.current_pose.pose.orientation

				print "############# current_pose #############"
				print self.arm.get_current_pose()

				target_pose.pose.position.z -= .03
				# target_pose.pose.position.y += .01

				inter_pose = deepcopy(target_pose)
				inter_pose.pose.position.z += .05

				self.arm.set_pose_target(inter_pose)
				self.arm.go()

				self.arm.set_pose_target(target_pose)
				self.arm.go()
				rospy.sleep(.05)

if __name__ == "__main__":
	Gatlin_Server()