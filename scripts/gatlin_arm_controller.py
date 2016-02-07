#!/usr/bin/env python

import rospy, sys, tf
import moveit_commander
from math import *
from geometry_msgs.msg import PoseStamped, Pose, Vector3, Quaternion, Point, PointStamped, Vector3Stamped, Vector3
from std_msgs.msg import String, Float64, Header
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from visualization_msgs.msg import Marker
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionIKRequest, GetPositionIKResponse, GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import RobotTrajectory, PositionIKRequest, RobotState, DisplayRobotState
# from moveit_msgs.msg import KinematicSolverInfo, PositionIKRequest
# from moveit_msgs.srv import GetKinematicSolverInfo, GetPositionIK, GetPositionIKRequest
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from tf.transformations import quaternion_from_euler
from tf import TransformListener

from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'gripper_link'
GRIPPER_JOINT_NAMES = ['gripper_joint']
GRIPPER_EFFORT = [1.0]
GRIPPER_PARAM = '/gripper_controller'

REFERENCE_FRAME = '/base_link'
ARM_BASE_FRAME = '/arm_base_link'

waypoints = []
done = True


class Gatlin_Server:
	def __init__(self):
		global initialPose
		global initialPoseR
		global arm
		global gripper
		global scene
		global end_effector_link
		global current_pose
		global rest_pose
		global gripper_pub
		global get_ik_proxy
		global robot

		global tf

		# global done
		# global waypoints
		global jt
		jt = JointTrajectory()

		# Give the launch a chance to catch up
		rospy.sleep(5)
		print "Launched Gatlin Server"
		# roscpp_initialize(sys.argv)
		# rospy.init_node('Gatlin_Server')

		# Initialize the move_group API
		moveit_commander.roscpp_initialize(sys.argv)

		rospy.init_node('Gatlin_Server')

		tf = TransformListener()

		self.gripper_opened = [rospy.get_param(GRIPPER_PARAM + "/max_opening") - 0.001]
		self.gripper_closed = [rospy.get_param(GRIPPER_PARAM + "/min_opening") + 0.001]
		self.gripper_neutral = [rospy.get_param(GRIPPER_PARAM + "/neutral",
			                                (self.gripper_opened[0] + self.gripper_closed[0])/2.0) ]

		self.gripper_tighten = rospy.get_param(GRIPPER_PARAM + "/tighten", 0.0) 

		# We need a tf listener to convert poses into arm reference base
		# tf = tf.TransformListener()

		# Use the planning scene object to add or remove objects
		scene = PlanningSceneInterface()

		# Create a scene publisher to push changes to the scene
		self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)

		# Create a publisher for displaying gripper poses
		self.gripper_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)

		# init a gripper publisher because movegroup won't work
		gripper_pub = rospy.Publisher('/gripper_joint/command', Float64)

		# Create a dictionary to hold object colors
		self.colors = dict()

		# Initialize the move group for the right arm
		arm = MoveGroupCommander(GROUP_NAME_ARM)

		robot = moveit_commander.RobotCommander()

		# Initialize the move group for the right gripper
		gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)

		
		# Get the name of the end-effector link
		end_effector_link = arm.get_end_effector_link()

		# Allow replanning to increase the odds of a solution
		arm.allow_replanning(True)

		# Set the planner
		# arm.set_planner_id("RRTstarkConfigDefault")
		arm.set_planner_id("RRTConnectkConfigDefault")

		# Set the right arm reference frame
		arm.set_pose_reference_frame(REFERENCE_FRAME)

		# Give each of the scene objects a unique name
		table_id = 'table'
		kinect_id = 'kinect'
		# box1_id = 'box1'
		# box2_id = 'box2'
		# target_id = 'target'
		# tool_id = 'tool'

		# Remove leftover objects from a previous run
		scene.remove_world_object(table_id)
		scene.remove_world_object(kinect_id)
		# scene.remove_world_object(box1_id)
		# scene.remove_world_object(box2_id)
		# scene.remove_world_object(target_id)
		# scene.remove_world_object(tool_id)

		# Remove any attached objects from a previous session
		# scene.remove_attached_object(GRIPPER_FRAME, target_id)



		# Give the scene a chance to catch up
		rospy.sleep(1)

		print arm.get_current_pose()

		# Start the arm in the "arm_up" pose stored in the SRDF file
		# rospy.loginfo("Set Arm: right_up")
		# arm.set_named_target('right_up')
		# if arm.go() != True:
		#     rospy.logwarn("Go failed")
		rospy.sleep(1)

		# # Move the gripper to the closed position
		# rospy.loginfo("Set Gripper: Close " + str(self.gripper_closed ) )
		# gripper.set_joint_value_target(self.gripper_closed)   
		# if gripper.go() != True:
		#     rospy.logwarn("  Go failed")
		# rospy.sleep(2)

		# # Move the gripper to the neutral position
		# rospy.loginfo("Set Gripper: Neutral " + str(self.gripper_neutral) )
		# gripper.set_joint_value_target(self.gripper_neutral)
		# if gripper.go() != True:
		#     rospy.logwarn("  Go failed")
		# rospy.sleep(2)

		# # Move the gripper to the open position
		# rospy.loginfo("Set Gripper: Open " +  str(self.gripper_opened))
		# gripper.set_joint_value_target(self.gripper_opened)
		# if gripper.go() != True:
		#     rospy.logwarn("  Go failed")
		# rospy.sleep(2)

		# Set the height of the table off the ground
		# table_ground = 0.44
		# table_ground = 0.33
		table_ground = 0.0

		# Set the dimensions of the scene objects [l, w, h]
		table_size = [1.0,1.0, 0.001]

		# Add a table top and two boxes to the scene
		table_pose = PoseStamped()
		table_pose.header.frame_id = REFERENCE_FRAME
		table_pose.pose.position.x = 0.0
		table_pose.pose.position.y = 0.0
		table_pose.pose.position.z = table_ground
		table_pose.pose.orientation.w = 1.0
		# scene.add_box(table_id, table_pose, table_size)

		# Set the height of the kinect off the ground
		kinect_ground = 0.44

		# Set the dimensions of the scene objects [l, w, h]
		kinect_size = [0.04, 0.12, 0.04]

		# Add a kinect top and two boxes to the scene
		kinect_pose = PoseStamped()
		kinect_pose.header.frame_id = REFERENCE_FRAME
		kinect_pose.pose.position.x = Point(0.28, 0.03, 0.78)

		# shoulder_lift_pos = [0.1, 0.03, 0.45]

		q = quaternion_from_euler(0, -0.78, 0) # pi/4 = .78
		kinect_pose.pose.orientation.x = q[0]
		kinect_pose.pose.orientation.y = q[1]
		kinect_pose.pose.orientation.z = q[2]
		kinect_pose.pose.orientation.w = q[3]
		# scene.add_box(kinect_id, kinect_pose, kinect_size)

		# print kinect_pose.pose

		# Make the table red and the boxes orange
		self.setColor(table_id, 0.8, 0, 0, .75)
		# self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
		# self.setColor(box2_id, 0.8, 0.4, 0, 1.0)

		# # Specify a pose to place the target after being picked up
		# place_pose = PoseStamped()
		# place_pose.header.frame_id = REFERENCE_FRAME
		# place_pose.pose.position.x = table_pose.pose.position.x - 0.03
		# place_pose.pose.position.y = -0.15
		# place_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
		# place_pose.pose.orientation.w = 1.0

		# # Initialize the grasp pose to the target pose
		# grasp_pose = target_pose

		# # Shift the grasp pose by half the width of the target to center it
		# grasp_pose.pose.position.y -= target_size[1] / 2.0

		
		# rospy.loginfo("Set Arm: down")
		# arm.set_named_target('down')
		# arm.go()
		# rospy.sleep(1)

		# Allow some leeway in position (meters) and orientation (radians)
		# USELESS; do not work on pick and place! Explained on this issue:
		# https://github.com/ros-planning/moveit_ros/issues/577
		arm.set_goal_position_tolerance(0.01)
		arm.set_goal_orientation_tolerance(0.2)

		# Allow 2 seconds per planning attempt
		arm.set_planning_time(2.0)


		# Create a quaternion from the Euler angles
		#q = quaternion_from_euler(0, pitch, yaw)

		# horiz = Quaternion(-0.023604, 0.99942, 0.00049317, 0.024555)
		# deg45 = Quaternion(-0.022174, 0.9476, 0.0074215, 0.31861)
		down = Quaternion(-0.00035087, 0.73273, 0.00030411, 0.68052)
		# back_pos = Point(-0.03, 0.0313, 0.476)


		rest_pose = PoseStamped()
		rest_pose.header.frame_id = REFERENCE_FRAME
		# rest_pose.pose.position = Point(-0.11717, 0.03333, 0.58279)
		# rest_pose.pose.orientation = Quaternion(0.0076628, 0.9990650, -0.0003283, 0.0425465)
		# rest_pose.pose.position = Point(-0.0913464322209, -0.00226331319848, 0.289462918772)
		# rest_pose.pose.orientation = Quaternion(-0.00490691506874, 0.959525059684, 0.00143787463608, 0.281576835941)
		rest_pose.pose.position = Point(-0.0762700684061, 0.0105568131039, 0.290791199591)
		rest_pose.pose.orientation = Quaternion(0.0251355325061, 0.982948881414, -0.0046583987932, 0.182093384981)


		# rospy.loginfo("Set Arm: down")
		# arm.set_named_target('down')
		# arm.go()
		# rospy.sleep(1)

		# current_pose = arm.get_current_pose()

		current_pose = PoseStamped()
		current_pose.header.frame_id = REFERENCE_FRAME
		current_pose.pose.position = Point(0,0,0)
		current_pose.pose.orientation = down


		# rospy.loginfo("Set Gripper: closed")
		# gripper_set(0.0)

		# rospy.sleep(1)

		# Open the gripper
		rospy.loginfo("Set Gripper: open")
		gripper_set(1.0)

		arm.set_pose_target(rest_pose)
		# arm.go()
		rospy.sleep(1)

		# current_pose = start_pose

		
		
		# gripper.set_named_target('grip_closed')
		# gripper.set_joint_value_target([0.002])
		# gripper.go()
		

		# while(True):
		# 	gripper_set(0.0)
		# 	rospy.sleep(2)
		# 	gripper_set(1.0)
		# 	rospy.sleep(2)





		# gripper.set_joint_value_target(-1.0)
		# gripper.go()

		#rospy.sleep(1)

		# Shut down MoveIt cleanly
		#moveit_commander.roscpp_shutdown()

		# Exit the script
		#moveit_commander.os._exit(0)

		# Create a quaternion from the Euler angles
		#q = quaternion_from_euler(0, pitch, yaw)

		# Set the place pose orientation accordingly
		#place.pose.orientation.x = q[0]
		#place.pose.orientation.y = q[1]
		#place.pose.orientation.z = q[2]
		#place.pose.orientation.w = q[3]

		# Append this place pose to the list
		#places.append(deepcopy(place))

		# waypoints = []
		rospy.Subscriber("/arm_target_pose", Pose, self.arm_target_pose_cb, queue_size=1)


		rospy.Subscriber("/target_pos", Vector3, pos_callback, queue_size=1)
		# rospy.Subscriber("/target_arm_pose", Float32MultiArray, pose_cb, queue_size=1)
		rospy.Subscriber("/target_orientation", Vector3, orientation_cb, queue_size=1)
		# # rospy.Subscriber("/target_go", String, go_callback, queue_size=1)

		# connect to arm kinematics
		IK_SERVICE_NAME = '/compute_ik'
		# rospy.loginfo("Waiting for service " + IK_SERVICE_NAME)
		rospy.wait_for_service(IK_SERVICE_NAME)
		get_ik_proxy = rospy.ServiceProxy(IK_SERVICE_NAME, GetPositionIK)

		# traj_pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', Float64)


		# rospy.wait_for_service('moveit/get_ik')
		# rospy.wait_for_service('moveit/get_ik_solver_info')
		# get_ik_proxy = rospy.ServiceProxy('moveit/get_ik', GetPositionIK, persistent=True)
		# get_ik_solver_info_proxy = rospy.ServiceProxy('moveit/get_ik_solver_info', GetKinematicSolverInfo)

		# topic = 'visualization_marker'
		# marker_pub = rospy.Publisher(topic, Marker)




		# pos_callback(Vector3(1,1,1))
		# pos_callback(Vector3(1,1,1))
		# pos_callback(Vector3(1,1,1))


		

		rospy.spin()


	def arm_target_pose_cb(self, msg):
		# try:
		print "Entering arm_target_pose_cb Callback"
		base_rel_pose = kinect_to_base(msg.position)
		print "RETRUNED FROM KINECT TO BASE"
		global REFERENCE_FRAME

		arm_target_pose = PoseStamped()
		arm_target_pose.header.frame_id = REFERENCE_FRAME
		arm_target_pose.header.stamp = rospy.Time.now() 
			# print arm.get_current_pose()
		down = Quaternion(-0.00035087, 0.73273, 0.00030411, 0.68052)
		arm_target_pose.pose.position = deepcopy(base_rel_pose)
		arm_target_pose.pose.orientation = down
		#arm_target_pose.pose.position.z -= .0

		inter_pose = deepcopy(arm_target_pose)
		inter_pose.pose.position.z += .05

		arm.set_pose_target(inter_pose)
		arm.go()
		
		arm.set_pose_target(arm_target_pose)

		#self.arm_target_pose_pub.publish(arm_target_pose)
		arm.go()


		rospy.sleep(1)
		print "finished arm_target_pose_cb"


	# Set the color of an object
	def setColor(self, name, r, g, b, a):
		# Initialize a MoveIt color object
		color = ObjectColor()

		# Set the id to the name given as an argument
		color.id = name

		# Set the rgb and alpha values given as input
		color.color.r = r
		color.color.g = g
		color.color.b = b
		color.color.a = a

		# Update the global color dictionary
		self.colors[name] = color

	# Actually send the colors to MoveIt!
	def sendColors(self):
		# Initialize a planning scene object
		p = PlanningScene()

		# Need to publish a planning scene diff
		p.is_diff = True

		# Append the colors from the global color dictionary
		for color in self.colors.values():
			p.object_colors.append(color)

		# Publish the scene diff
		self.scene_pub.publish(p)

def gripper_set(val):
	global gripper_pub
	gripper_pub.publish((1-val)*-2.53)

def pose_cb(data):
	f = data.data
	print f

	# dim = MultiArrayDimension()
	# dim.size = len(arrayList)
	# dim.label = label
	# dim.stride = len(arrayList)

	# tempArray = Float64MultiArray()
	# tempArray.data = arrayList
	# tempArray.layout.dim.append(dim)
	# tempArray.layout.data_offset = 0


	# orientData = Vector3(-1.0,-2.0,f[3])
	# posData = Vector3(f[0],f[1],f[2])
	# orientation_cb(orientData)
	# pos_callback(posData)


	#takes a point from kinect to base frame
def kinect_to_base(data) :
	rospy.loginfo("kinect to base!: [%f, %f, %f]"%(data.x, data.y, data.z))
	
	
	shoulder_lift_pos = [0.1, 0.03, 0.44]
	target_pos = [data.x, -data.y, data.z]

	kinectPt = PointStamped()
	kinectPt.header.frame_id = "/camera_rgb_optical_frame"
	
	kinectPt.header.stamp = rospy.Time.now() - rospy.Duration(.22)
	kinectPt.point = Point(target_pos[0],target_pos[1],target_pos[2])

	basePt = tf.transformPoint("/base_link", kinectPt)

	print "############# basePt #############"
	print basePt
	return basePt.point

def orientation_cb(data):
	global current_pose
	if(data.x == -1.0 and data.y == -2.0):
			print "################################################"
			print "###################  Orientation!  #############"
			print "################################################"
			deg = data.z*15
			# rad = deg * pi/180 + pi/2
			rad = -deg * pi/180 + pi/2
			print rad
			q = quaternion_from_euler(0,rad,0)
			current_pose.pose.orientation.x = q[0]
			current_pose.pose.orientation.y = q[1]
			current_pose.pose.orientation.z = q[2]
			current_pose.pose.orientation.w = q[3]
	else:
		return

def pos_callback(data):
	global current_pose
	global rest_pose
	global done
	if done:
		if(data.x == 0.0 and data.y == 0.0 and data.z == 0.0):
			done = False
			print "################################################"
			print "###################  Going!  ###################"
			print "################################################"
			if arm.go() != True:
				rospy.logwarn("Go to target_pos failed")
			# rospy.sleep(.2)
			done = True

		elif(data.x == -1.0 and data.y == -1.0):
			return		
		elif(data.x == -1.0 and data.y == -2.0):
			return
		elif(data.x == -2.0 and data.y == -2.0 and data.z == -2.0):
			print "################################################"
			print "###################  Reset!  ###################"
			print "################################################"

			arm.set_pose_target(rest_pose)
			arm.go()

			rospy.loginfo("Set Gripper: open")
			#gripper_set(1.0)

			rospy.sleep(1)
			return		
		else:

			rospy.loginfo("Received a target_pos message: [%f, %f, %f]"%(data.x, data.y, data.z))

			# add up to 10 points to cartesian path
			
			# q = Quaternion(0.0017, 0.6734,-0.0019,0.7393)

			scale = 3000.0
			scale = 1.0
			
			shoulder_lift_pos = [0.1, 0.03, 0.44]

			# target_pos = [data.z/scale, data.y/scale, data.x/scale]
			target_pos = [data.x, -data.y, data.z]

			# target_pos = [target_pos[0] + shoulder_lift_pos[0], 
			# target_pos[1] + shoulder_lift_pos[1], 
			# target_pos[2] + shoulder_lift_pos[2], ]

			# back_target_pos = [-data.z/scale, -data.y/scale, data.x/scale]

			# back_target_pos = [back_target_pos[0] + shoulder_lift_pos[0], 
			# back_target_pos[1] + shoulder_lift_pos[1], 
			# back_target_pos[2] + shoulder_lift_pos[2], ]

			# print "############# target_pos #############"
			# print target_pos

			kinectPt = PointStamped()
			kinectPt.header.frame_id = "/camera_rgb_optical_frame"
			# kinectPt.header.frame_id = "/arm_target_frame"
			kinectPt.header.stamp = rospy.Time.now() - rospy.Duration(.22)
			kinectPt.point = Point(target_pos[0],target_pos[1],target_pos[2])

			basePt = tf.transformPoint("/base_link", kinectPt)

			print "############# basePt #############"
			print basePt
			# target_pos = [basePt.point.x,basePt.point.y,basePt.point.z]


			# if tf.frameExists("/base_link") and self.tf.frameExists("/camera_rgb_optical_frame"):
				# t = tf.getLatestCommonTime("/base_link", "/camera_rgb_optical_frame")
				# position, quaternion = tf.lookupTransform("/base_link", "/camera_rgb_optical_frame", t)
				# p = PointStamped()
				# print position, quaternion

			# pose = Pose()
			# pose.orientation = current_pose.pose.orientation
			# pose.position.x = data.z/scale
			# pose.position.y = data.y/scale
			# pose.position.z = data.x/scale

			# global prev

			# isFar = False
			
			# try:
			# 	dist = sqrt(
			# 	pow((back_target_pos[0] - prev[0]), 2) + 
			# 	pow((back_target_pos[1] - prev[1]), 2) +
			# 	pow((back_target_pos[2] - prev[2]), 2))

			# 	# print '############################################'
			# 	# print '############################################'
			# 	# print dist
			# 	# print '############################################'
			# 	# print '############################################'

			# 	isFar = dist > 0.007

			# except:
			# 	prev = back_target_pos

			# global waypoints
			# global jt

			# arm.set_pose_target(pose)

			# print "target_pos: "
			# print target_pos

			# arm.set_position_target(target_pos)
			# go_callback("null")



			# # Create a contraints list and give it a name
			# constraints = Constraints()
			# constraints.name = "Gripper horizontal"

			# # Create an orientation constraint for the right gripper 
			# orientation_constraint = OrientationConstraint()
			# orientation_constraint.header.frame_id = REFERENCE_FRAME
			# orientation_constraint.link_name = end_effector_link
			# orientation_constraint.orientation.x = 0.0
			# orientation_constraint.orientation.y = 0.673
			# orientation_constraint.orientation.z = 0.0
			# orientation_constraint.orientation.w = 0.739
			# # orientation_constraint.orientation = current_pose.pose.orientation
			# print orientation_constraint.orientation

			# # orientation_constraint.absolute_roll_tolerance = 0.3;
			# # orientation_constraint.absolute_pitch_tolerance = 0.3;
			# # orientation_constraint.absolute_yaw_tolerance = 3.14;

			# orientation_constraint.absolute_x_axis_tolerance = 3.14
			# orientation_constraint.absolute_y_axis_tolerance = 3.14
			# orientation_constraint.absolute_z_axis_tolerance = 3.14
			# # orientation_constraint.weight = 1.0

			# # Append the constraint to the list of contraints
			# constraints.orientation_constraints.append(orientation_constraint)

			# Set the path constraints on the right_arm
			# arm.set_path_constraints(constraints)

			# Set a target pose for the arm        
			target_pose = PoseStamped()
			target_pose = current_pose

			# print current_pose.pose.orientation
			target_pose.pose.position = basePt.point
			# target_pose.pose.position.x = target_pos[0]
			# target_pose.pose.position.y = target_pos[1]
			# target_pose.pose.position.z = target_pos[2]
			# target_pose.pose.position.x = back_target_pos[0]
			# target_pose.pose.position.y = back_target_pos[1]
			# target_pose.pose.position.z = back_target_pos[2]

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
			target_pose.pose.orientation = current_pose.pose.orientation

			# print current_pose.pose.orientation

			

			# target_pose.pose.position.x -= 0
			# target_pose.pose.position.y -= .05
			# target_pose.pose.position.z += .05


			# rospy.loginfo("Set Arm: down")
			# arm.set_named_target('down')
			# arm.go()
			# rospy.sleep(1)

			print "############# current_pose #############"
			print arm.get_current_pose()

			# print "############# target_pose #############"
			# print target_pose

			target_pose.pose.position.z -= .03
			# target_pose.pose.position.y += .01

			inter_pose = deepcopy(target_pose)
			inter_pose.pose.position.z += .05

			arm.set_pose_target(inter_pose)
			arm.go()


			# gripperDown = Vector3Stamped()
			# gripperDown.header.frame_id = "/gripper_servo_link"
			# gripperDown.vector = Vector3(0,.03,0)

			# tf.waitForTransform("/gripper_servo_link", "/base_link", rospy.Time(), rospy.Duration(4.0))
			# gripperDown.header.stamp = rospy.Time.now()
			# gripperDown.header.stamp -= rospy.Duration(5.5)
			# baseGripperDown = tf.transformVector3("/base_link", gripperDown)
			# print baseGripperDown.vector
			# rospy.sleep(.05)

			# target_pose.pose.position.x += baseGripperDown.vector.x
			# target_pose.pose.position.y += baseGripperDown.vector.y
			# target_pose.pose.position.z += baseGripperDown.vector.z

			# get the the positive y direction in the 'gripper_servo_link' frame
			# transform it to the base_link frame and add it to the target position

			arm.set_pose_target(target_pose)
			arm.go()
			rospy.sleep(.05)

			
			# if isFar:
			# 	prev = back_target_pos

			# 	ik_answer = getIkPose(target_pose.pose)
			# 	# print ik_answer


			# 	jtp = JointTrajectoryPoint()
			# 	jtp.positions = ik_answer.solution.joint_state.position[:-2]
			# 	# vel = 0.1
			# 	# jtp.velocities = [vel, vel, vel, vel, 0.0]
			# 	# acc = 1.5
			# 	# jtp.accelerations = [acc] * 4
			# 	# effort = 10.0
			# 	# jtp.effort = [effort, effort, effort, effort, 0.0]
			# 	dur = 0.8*(len(jt.points)+1)*sqrt(sqrt(dist))/.6
			# 	jtp.time_from_start = rospy.Duration(dur)
			# 	# print '#######################################'
			# 	# print '#######################################'
			# 	# print dur
			# 	# print '#######################################'
			# 	# print '#######################################'


			# 	# jt = JointTrajectory()
			# 	jt.joint_names = ik_answer.solution.joint_state.name[:-2]
			# 	jt.points.append(deepcopy(jtp))
			# 	jt.header.stamp = rospy.Time.now()
			# 	jt.header.frame_id = REFERENCE_FRAME
			# 	# print jt.joint_names

			# 	# TODO:
			# 	# Open button stops working but close still works
			# 	# gripper keeps getting reset to original position
			# 	# ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 
			# 	# 'gripper_joint']

			# 	# fjt_goal = FollowJointTrajectoryGoal()
			# 	# fjt_goal.trajectory = jt
			# 	# print fjt_goal

			# 	if(len(jt.points) >= 1):
			# 		# print '#######################################'
			# 		# print '#######################################'
			# 		# print jt.points
			# 		# print '#######################################'
			# 		# print '#######################################'

			# 		# Set the start state and target pose, then plan and execute
			# 		arm.set_start_state_to_current_state()

			# 		rt = RobotTrajectory()
			# 		rt.joint_trajectory = jt
			# 		arm.execute(rt)
			# 		rospy.sleep(.1)

			# 		jt = JointTrajectory()

			# # waypoints = []

			# # # start with the current pose
			# # waypoints.append(group.get_current_pose().pose)

			# # # first orient gripper and move forward (+x)
			# # wpose = Pose()
			# # wpose.orientation.w = 1.0
			# # wpose.position.x = waypoints[0].position.x + 0.1
			# # wpose.position.y = waypoints[0].position.y
			# # wpose.position.z = waypoints[0].position.z
			# # waypoints.append(deepcopy(target_pose.pose))

			# # # second move down
			# # wpose.position.z -= 0.10
			# # waypoints.append(copy.deepcopy(wpose))

			# # # third move to the side
			# # wpose.position.y += 0.05
			# # waypoints.append(copy.deepcopy(wpose))

			# # waypoints.append(deepcopy(target_pose.pose))

			# # if(len(waypoints) >= 2):
			# # 	# print waypoints

			# # 	# plan cartesian path with waypoints
			# # 	(plan, fraction) = arm.compute_cartesian_path(
			# # 		waypoints,   # waypoints to follow
			# # 		0.01,        # eef_step
			# # 		0.0)         # jump_threshold

			# # 	arm.execute(plan)
			# # 	# print '#######################################'
			# # 	# print '#######################################'
			# # 	# print(plan)
			# # 	# print '#######################################'
			# # 	# print '#######################################'

			# # 	waypoints = [] # empty list

			# 	# print "============ Waiting while RVIZ displays plan..."
			# 	# rospy.sleep(2)

			# 	# # arm.computeCartesianPath(waypoints)
			# 	# # arm.go()
			# 	# ekp = rospy.ServiceProxy('/execute_kinematic_path', ExecuteKnownTrajectory)
			# 	# ekp.wait_for_service()

			# 	# print "!!!! Gonna send goal to execute_kinematic_path (waiting 3s)"
			# 	# rospy.sleep(3)
			# 	# ektr = ExecuteKnownTrajectoryRequest()
			# 	# ektr.trajectory = plan3
			# 	# ektr.wait_for_execution = True
			# 	# print "Sending call"
			# 	# ekp.call(ektr)
			# 	# print "!!!! Call done"



			

			# # out_limit_pos = [0.421, 0.03, 0.4955]


			# # forward_pos = [0.26943, 0.03, 0.76349]

			# # current_pose = arm.get_current_pose()
			# # #print current_pose
			# # rospy.sleep(1)

			# # down_pos = [0.27939, 0.03, 0.42694]
			# # arm.set_position_target(down_pos)
			# # #arm.go()
			# # rospy.sleep(1)

			# # target_pos = [0.2, -0.1, 0.0]


			# # if arm.go() != True:
			# #     rospy.logwarn("Go to target_pos failed")
			# # rospy.sleep(.5)


			# # Given a set of x,y,z coord, move to that position or return 0 if no solution

			# # get the shoulder_lift xyz (shoulder_lift_pos)
			# # set up a topic to listen to for the new target
			# # get 3d vector coming from Vuforia (target_pos)
			# # then set move group target to target_pos + shoulder_lift_pos
			# # display the Vuforia pos in rviz
			# # If out of range return error

			
			# # create ikfast solution to check
			# # or maybe get solution from rviz
			# # add the kinect to the planning scene as an object

			# # points = 
			# # pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)
			
			# # PositionIKRequest

			# # jt = JointTrajectory()

			# # jt.header.stamp = rospy.Time.now()
			# # jt.header.frame_id = REFERENCE_FRAME
			# # jt.joint_names = ['arm_shoulder_pan_joint', 'arm_shoulder_lift_joint', 'arm_elbow_flex_joint', 'arm_wrist_flex_joint', 'gripper_link_joint']
			# # jt.points = points

# def position_ik_client(x, y):
# 	rospy.wait_for_service('add_two_ints')
# 	try:
# 		add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
# 		resp1 = add_two_ints(x, y)
# 		return resp1.sum
# 	except rospy.ServiceException, e:
# 		print "Service call failed: %s"%e
def getIkPose(pose, previous_state=None):
	"""Get IK of the pose specified, for the group specified, optionally using
	the robot_state of previous_state (if not, current robot state will be requested) """
	global get_ik_proxy
	global robot
	# point point to test if there is ik
	# returns the answer of the service
	rqst = GetPositionIKRequest()
	# rqst.ik_request.avoid_collisions = False
	rqst.ik_request.avoid_collisions = True
	rqst.ik_request.group_name = 'arm'
	rqst.ik_request.pose_stamped.header = Header(stamp=rospy.Time.now())
	rqst.ik_request.pose_stamped.header.frame_id = REFERENCE_FRAME
	# rqst.timeout = rospy.Duration(5.0)


	# Set point to check IK for
	rqst.ik_request.pose_stamped.pose.position = pose.position
	rqst.ik_request.pose_stamped.pose.orientation = pose.orientation

	if previous_state == None:
		cs = robot.get_current_state()
		rqst.ik_request.robot_state = cs
	else:
		rqst.ik_request.robot_state = previous_state

	ik_answer = GetPositionIKResponse()
	timeStart = rospy.Time.now()
	ik_answer = get_ik_proxy.call(rqst)
	durationCall= rospy.Time.now() - timeStart
	rospy.loginfo("Call took: " + str(durationCall.to_sec()) + "s")

	return ik_answer

# def get_ik(target, group = "arm"):
# 	# arm.set_pose_target(target_pose)
# 	# arm.go()
# 	# rospy.sleep(.05)

# 	# create IK request
# 	request = GetPositionIKRequest()
# 	request.timeout = rospy.Duration(self.timeout)

# 	request.ik_request.pose_stamped.header.frame_id = REFERENCE_FRAME;
# 	request.ik_request.ik_link_name = 'GRIPPER_FRAME';
# 	request.ik_request.avoid_collisions = False
# 	request.ik_request.pose_stamped.pose.position.x = target_pose.pose.position.x
# 	request.ik_request.pose_stamped.pose.position.y = target_pose.pose.position.y
# 	request.ik_request.pose_stamped.pose.position.z = target_pose.pose.position.z
# # 	service_request = PositionIKRequest()
# # 	service_request.group_name = group
# # 	#service_request.robot_state = initial_state
# # 	#service_request.ik_link_name = "pen_link"
# # 	service_request.pose_stamped = target
# # 	service_request.timeout.secs= 0.1
# # 	service_request.avoid_collisions = False

# 	# try:
# 	# 	resp = compute_ik(ik_request = service_request)
# 	# 	return resp
# 	# except rospy.ServiceException, e:
# 	# 	print "Service call failed: %s"%e

if __name__ == "__main__":
	Gatlin_Server()