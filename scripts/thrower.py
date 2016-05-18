#!/usr/bin/env python
import rospy
import math
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from cs4752_proj3.msg import *
from cs4752_proj3.srv import *
from baxter_core_msgs.msg import *
from baxter_core_msgs.srv import *
import baxter_interface
from baxter_interface import *
import baxter_external_devices
from config import *
from copy import deepcopy
from tf.transformations import *
import baxter_pykdl

# source: http://sdk.rethinkrobotics.com/wiki/Hardware_Specifications

# Maximum Joint Speeds (rad/sec)
# Joint	Maximum Speed
# S0	2.0
# S1	2.0
# E0	2.0
# E1	2.0
# W0	4.0
# W1	4.0
# W2	4.0

# Joint Range Table
# Joint	Min limit	Max limit	Range (Radians)
# S0	-2.461		+0.890		3.351
# S1	-2.147		+1.047		3.194
# E0	-3.028		+3.028		6.056
# E1	-0.052		+2.618		2.67
# W0	-3.059		+3.059		6.117
# W1	-1.571		+2.094		3.665
# W2	-3.059		+3.059		6.117

class thrower :
	def __init__(self):
		rospy.init_node('thrower')
		rospy.loginfo("Initialized Thrower")

		rs = baxter_interface.RobotEnable(CHECK_VERSION)
		init_state = rs.state().enabled
		self.baxter = RobotEnable()


		# self.limb = rospy.get_param("limb")
		self.limb = 'left'
		self.robot_name = "baxter"
		self.limb_name = self.limb
		
		self.move_arm = createServiceProxy("/%s_%s/move/arm" % (self.robot_name, self.limb_name), MoveRobot)

		self.gripper = baxter_interface.Gripper(self.limb, CHECK_VERSION)

		self.arm = Limb(self.limb)

		self.arm_kin = baxter_pykdl.baxter_kinematics(self.limb)

		self.target_pose_pub = rospy.Publisher('zic_target_pose', PoseStamped, queue_size=1)

		throw_service = createService('throw', Action, self.throw_srv)
		
		# set the release pose
		# release_pose = Pose()
		# release_pose.position = release_pos
		# release_pose.orientation = Quaternion(0.140770659119,0.989645234506,0.0116543447684,0.0254972076605)
		# success = self.move_arm(MOVE_TO_POSE, self.limb, release_pose)
		
		# move to the release pose
		# self.moveToThrow()

		# recorded good throw configs:
		# self.moveToThrow(w1=-0.7, w0=0.5)
		# self.moveToThrow(w1=-0.7, e1=2.40)
		# self.moveToThrow(w1=-0.7, e1=1.57)

		self.moveToThrow(w1=-0.22, w0=-0.2, e1=1.69)

		# self.arm.move_to_neutral()

		# rospy.logerr(self.arm.joint_angles())
		# rospy.logerr(self.arm.joint_efforts())
		# rospy.logerr(self.arm.joint_effort("left_w1"))


		
		# close the gripper
		# self.move_arm(CLOSE_GRIPPER, self.limb, Pose())

		# throw the ball
		self.throw()
		# self.throw_2([0.5, 0.0, 0.1], 1.0, 0.0, math.radians(10))

		# torques = {'left_w0': 0.0, 'left_w1': 0.0, 'left_w2': 0.0, 'left_e0': 0.0, 'left_e1': 0.0, 'left_s0': 0.0, 'left_s1': 0.0}

		# rate = rospy.Rate(30)
		# joint_efforts = self.arm.joint_efforts()
		# while not rospy.is_shutdown():
			# rospy.logerr(self.arm.joint_efforts())
			# self.arm.set_joint_torques(torques)


		#rospy.spin()

	def throw_srv(self, req):
		if self.canceled:
			return ActionResponse(False)
		self.moveToThrow(w1=-0.7)

		if self.canceled:
			return ActionResponse(False)
		self.throw()

		return ActionResponse(True)

	def moveToThrow(self, e1=2.222, w0=1.7357, w1=0.0000) :
		e1_range = [1.57, 2.22, 2.40] # elbow far or close to body
		w0_range = [0.92, 1.74, 2.50] # yaw for bank shots
		w1_range = [-1.0, 0.00, 0.50] # release pitch

		common_pos = {
			's0': -0.90, 
			's1': -0.97,
			'e0': 0.11, 
			'w2': 0.00
		}

		new_pos = {}
		for key in common_pos:
			new_pos[self.limb+"_"+key] = common_pos[key]

		new_pos[self.limb+"_e1"] = e1
		new_pos[self.limb+"_w0"] = w0
		new_pos[self.limb+"_w1"] = w1

		if self.limb == 'right':
			new_pos = mirror_left_arm_joints(new_pos)

		self.gripper.command_position(100, block=True)
		rospy.logerr(new_pos)
		self.arm.move_to_joint_positions(new_pos)
		rospy.sleep(1)
		self.gripper.command_position(0, block=True)

	def throw(self) :
		# test throw params
		release_pos = Point(0.573, 0.081, 0.036)
		release_vel = Vector3(1.50, 0.00, 0.00)
		
		safty_buffer = math.radians(10.0)
		w1_min = -1.571
		w1_max = 2.094

		linear_speed = np.linalg.norm(vector3_to_numpy(release_vel))
		print "linear_speed (m/sec)"
		print linear_speed

		link_len = 0.31 # need to measure
		angular_speed = linear_speed/link_len
		if angular_speed > 4.0:
			angular_speed = 4.0
		# print "angular_speed (deg/sec)"
		# print  math.degrees(angular_speed)
		print "angular_speed (rad/sec)"
		print  angular_speed

		# throw = [0,0,0,0,0,-angular_speed,0]
		throw = [0,0,0,0,0,-0.79,0]
		zero = [0,0,0,0,0,0,0]
		throw_dict = {}
		zero_dict = {}

		for i in xrange(0,7) :
			throw_dict[self.limb+'_'+joint_names[i]] = throw[i]
			zero_dict[self.limb+'_'+joint_names[i]] = 0

		back_swing = math.radians(70.0)
		follow_through = math.radians(60.0)
		open_gripper_time = .10 # need to calibrate

		current_joints = self.arm.joint_angles()

		# set release_angle as current joint angle
		release_angle = current_joints[self.limb+'_w1']

		# set open_angle to compensate for open gripper time
		open_offset = open_gripper_time * angular_speed
		open_angle = release_angle + open_offset

		# set stop_angle allowing for follow through 
		stop_angle = release_angle - follow_through
		# make sure follow through won't hit joint limit
		if stop_angle < w1_min + safty_buffer :
			stop_angle = w1_min + safty_buffer

		# move the wrist bend to backswing pos
		# move the wrist twist to neutral pos
		current_joints[self.limb+'_w1'] += back_swing
		if current_joints[self.limb+'_w1'] > w1_max :
			current_joints[self.limb+'_w1'] = w1_max

		current_joints[self.limb+'_w2'] = 0.0
		self.arm.move_to_joint_positions(current_joints)
		rospy.sleep(.5)

		opened = False
		released = False
		rate = rospy.Rate(300)
		while True :
			current_joints = self.arm.joint_angles()
			current_angle = current_joints[self.limb+'_w1']

			# opens gripper slightly before release_angle to account for open gripper time
			if not opened and open_angle > current_angle : 
				self.gripper.command_position(100, block=False)
				opened = True
				# print "open_angle"
				# print open_angle
				# print "actual_open_angle"
				# print current_angle

			# should release at correct pos/vel
			if not released and release_angle > current_angle :
				actual_release_pos = np.array(self.arm.endpoint_pose()['position'])
				# actual_release_vel = np.array(self.arm.endpoint_velocity()['linear'])
				actual_release_speed = self.arm.joint_velocities()[self.limb+'_w1'] * link_len
				released = True
				print "actual_release_pos"
				print actual_release_pos
				print "actual_release_speed"
				print actual_release_speed

			# stops at the stop_angle
			if stop_angle > current_angle :
				self.arm.set_joint_velocities(zero_dict)
				break

			self.arm.set_joint_velocities(throw_dict)
			# rospy.logerr(self.arm.joint_efforts())
			rate.sleep()



if __name__ == '__main__':
	try:
		t = thrower()
	except rospy.ROSInterruptException:
		pass
