#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from geometry_msgs.msg import *
from tf.transformations import *

midfield_to_ball_start = 0.5334 # from field measurement
goal_width = 0.28
field_length = 1.38
field_width = 0.69
gripper_width = 0.05
gripper_depth = 0.03
ball_radius = 0.015

joint_names = ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']

CLOSE_GRIPPER = 0
OPEN_GRIPPER = 1
MOVE_TO_POSE = 2
MOVE_TO_POS = 3
MOVE_TO_POSE_INTERMEDIATE = 4

BLOCK = 10
GRAB = 13
THROW = 14
CHECK_BLOCKS = 15
MOVE_BLOCKS = 16

def createServiceProxy(service,srv_type,robot_name):
	name = ""
	if robot_name == "":
		name = "/%s" % (service)
	else:
		name = "/%s/%s" % (robot_name,service)
	rospy.wait_for_service(name)
	rospy.loginfo("Initialized service proxy for %s" % name)
	return rospy.ServiceProxy(name, srv_type)

def createService(service,srv_type,callback,robot_name):
	name = ""
	if robot_name == "":
		name = "/%s" % (service)
	else:
		name = "/%s/%s" % (robot_name,service)
	rospy.loginfo("Initialized service for %s" % name)
	return rospy.Service(name, srv_type, callback)

def vector3_to_numpy(v):
	return np.array([v.x, v.y, v.z])

def quaternion_to_numpy(q):
	return np.array([q.x, q.y, q.z, q.w])

def numpy_to_vector3(n):
	return Vector3(n[0],n[1],n[2])

def numpy_to_point(n):
	return Point(n[0],n[1],n[2])

def numpy_to_joint_dict(limb, data):
	joint_dict = {}
	for i in range(0,len(joint_names)):
		joint_dict[ limb + "_" + joint_names[i] ] = data[i]
	return joint_dict

def joint_dict_to_numpy(limb, joint_dict):
	numpy_array = np.empty(7)
	for i, joint_name in enumerate(joint_names):
		numpy_array[i] = joint_dict[limb + "_" + joint_name]
	return numpy_array

def quaternion_to_numpy(q):
	return np.array([q.x, q.y, q.z, q.w])
			# 	self.pub.publish(BlockerOffset(0.1))
			# else:
			# 	self.pub.pu

def invert_unit_quaternion(q):
		return np.array([-q[0], -q[1], -q[2], q[3]])

def multiply_quaternion(q_0, q_1):
	return np.array([ q_0[0]*q_1[3] + q_0[3]*q_1[0] - q_0[1]*q_1[2] + q_0[2]*q_1[1],
					   q_0[1]*q_1[3] + q_0[3]*q_1[1] - q_0[0]*q_1[2] + q_0[2]*q_1[0],
					   q_0[2]*q_1[3] + q_0[3]*q_1[2] - q_0[1]*q_1[0] + q_0[0]*q_1[1],
					   q_0[3]*q_1[3] - q_0[0]*q_1[0] - q_0[1]*q_1[1] - q_0[2]*q_1[2] ])

def mirror_left_arm_joints(new_pos):

	joint_limits = {
		"s0": [-2.461, 0.890],
		"s1": [-2.147, 1.047],
		"e0": [-3.028, 3.028],
		"e1": [-0.052, 2.618],
		"w0": [-3.059, 3.059],
		"w1": [-1.571, 2.094],
		"w2": [-3.059, 3.059]
	}

	joint_middle = {
		's0': 0.0, 
		's1': -0.55, 
		'e0': 0.0,
		'e1': 1.283, 
		'w0': 0.0, 
		'w1': 0.2615, 
		'w2': 0.0
	}
	for key in new_pos:
				limbless_key = key[-2:]
				if limbless_key != 'e1' and limbless_key != 's1' and limbless_key != 'w1':
					new_pos[key] = joint_middle[limbless_key] - (new_pos[key] - joint_middle[limbless_key])

	return new_pos

def null(a, rtol=1e-5):
	# http://stackoverflow.com/questions/19820921/a-simple-matlab-like-way-of-finding-the-null-space-of-a-small-matrix-in-numpy
	u, s, v = np.linalg.svd(a)
	rank = (s > rtol*s[0]).sum()
	return rank, v[rank:].T.copy()

def get_angular_error(desired, actual):
	relative_orientation = multiply_quaternion(actual, 
		invert_unit_quaternion(desired))
	relative_orientation_angle = 2.0*np.arccos(relative_orientation[3])
	if relative_orientation_angle < 1e-6:
		return np.zeros(3)
	else:
		relative_orientation_axis = relative_orientation[:3]/np.sin(0.5*relative_orientation_angle)
		orientation_error = relative_orientation_angle*relative_orientation_axis
		return orientation_error
	

def maximize_cosine_constrained(a,b,c,n2):
	# find t to maximize cosine between a t + b and c, with norm no greater than n2
	#
	# same as maximize_cosine but the result should have norm no greater than n^2
	# loginfo(a)
	# already normalized by the norm routine
	aa = np.dot(a,a)
	#loginfo(np.dot(a,a))
	#loginfo(b)
	ab = np.dot(a,b)
	#loginfo(ab)
	ac = np.dot(a,c)
	bc = np.dot(b,c)
	bb = np.dot(b,b)
	#loginfo("Computed Products")
	ab_over_aa = ab/aa
	lower_root = - ab_over_aa - np.sqrt(ab_over_aa**2 + (n2 - bb)/aa)
	upper_root = - ab_over_aa + np.sqrt(ab_over_aa**2 + (n2 - bb)/aa)

	s = (bc*ab - bb*ac)/(ab*ac - aa*bc)
	is_maximum = 2*ab*ac*s*s + (ab*ab*ac + 3*aa*ac*bb)*s + (aa*bc + 2*ab*ac)*bb > 2*bc*aa*aa*s*s + bc*ab*ab
	if is_maximum:
		#loginfo("Maximum")
		return np.clip(s, lower_root, upper_root)
	else:
		#loginfo("Minimum")
		upper_vec = a * upper_root + b
		lower_vec = a * lower_root + b
		upper_cos = np.dot(upper_vec,c)/np.sqrt(np.dot(upper_vec,upper_vec)*np.dot(c,c))
		lower_cos = np.dot(lower_vec,c)/np.sqrt(np.dot(lower_vec,lower_vec)*np.dot(c,c))
		if upper_cos > lower_cos:
			return upper_root
		else:
			return lower_root

def direction_of_manipulability(J, dJs, eps):
	J_squared = np.dot(J, J.T)
	J_squared_inv = np.linalg.inv(J_squared)

	direction_of_manipulability = np.empty(7)
	for i in xrange(0,7):

		dJ = (dJs[i] - J)/eps

		dJSquared = np.dot(J,dJ.T) + np.dot(dJ,J.T)

		trace = 0.0
		for j in xrange(0,6):
			for k in xrange(0,6):
				trace += J_squared_inv[j,k] * dJSquared[k,j]

		direction_of_manipulability[i] = trace
	return direction_of_manipulability

def get_current_pose(arm):
    p = Pose()
    pos = np.array(arm.endpoint_pose()['position'])
    ori = np.array(arm.endpoint_pose()['orientation'])
    p.position = Point(pos[0],pos[1],pos[2])
    p.orientation = Quaternion(ori[0],ori[1],ori[2],ori[3])
    return p

def getOffsetPose(self, pose, offset) :
    offsetpose = deepcopy(pose)
    offsetpose.position.x += offset.x
    offsetpose.position.y += offset.y
    offsetpose.position.z += offset.z
    return offsetpose

def getLocalOffsetPose(self, pose, offset) :
    offsetpose = deepcopy(pose)
    q = pose.orientation
    off = np.dot(
        quaternion_matrix([q.x,q.y,q.z,q.w]),
        np.array([offset.x,offset.y,offset.z,1]).T)
    off = off[:3]/off[3]
    offsetpose.position.x += off[0]
    offsetpose.position.y += off[1]
    offsetpose.position.z += off[2]
    return offsetpose
