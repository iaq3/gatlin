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
from baxter.msg import *
from baxter.srv import *
from config import *


def distance(v1,v2):
	return np.linalg.norm(vector3_to_numpy(v1) - vector3_to_numpy(v2))

def length(v):
	return math.sqrt(np.dot(v, v))

def angle(v1, v2):
	return math.acos(np.dot(v1, v2) / (length(v1) * length(v2)))

class DynamicPose:
	def __init__(self):
		self.FIXED_FRAME = "global_map"
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



	def grabObject(self, dynamic_pose) :
		

		#holding_object = False
		#while not holding_object :


		if self.command_state == self.CANCELLED :
			return
		self.pauseCommand()

		self.publishResponse("Attempting to grab "+dynamic_pose.topic)

		#open gripper
		#resp = self.move_arm(OPEN_GRIPPER, PoseStamped()) OPEN GRIPPER!!!
		#mr = MoveRobot()
		#mr.action = 1
		#mr.limb = "left"
		#mr.pose = Pose()
		resp = self.move_arm(1, self.limb, Pose())

		base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)
		#base_pose.pose.position.z -= .025 #??? maybe not best practice
		#resp = self.move_arm(MOVE_TO_POSE_INTERMEDIATE, base_pose)
		resp = self.move_arm(3, self.limb, base_pose.pose)

		if not resp.success:
			rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
			# try moving to it again
			#self.servoBaseToDynamicPos(self.object_pose)
			#TODO check this
			rospy.sleep(1)
			continue


		self.pauseCommand()

		#resp = self.move_arm(CLOSE_GRIPPER, PoseStamped())
		resp = self.move_arm(0, self.limb, Pose())

		self.pauseCommand()

		#resp = self.move_arm(RESET_ARM, PoseStamped())
		# base_pose.pose.position.z += .1
		resp = self.move_arm(3, self.limb, base_pose.pose)
		if not resp.success:
			rospy.logerr("RESET_ARM FAILED")

		# no object detection in last second, it is likely in robot's hand
		#if time.time() - dynamic_pose.last_update > 1 :
		#holding_object = True

		self.publishResponse("Grabbed "+dynamic_pose.topic)

	def pauseCommand(self) :
		while self.command_state == self.PAUSING :
			rospy.sleep(.03)

	def releaseObject(self, dynamic_pose) :
		if self.command_state == self.CANCELLED :
			return

		self.pauseCommand()

		self.publishResponse("Releasing object to "+dynamic_pose.topic)
		base_pose = self.transform_pose(self.BASE_FAME, dynamic_pose.ps)
		#resp = self.move_arm(MOVE_TO_POSE_INTERMEDIATE, base_pose)
		resp = self.move_arm(3, self.limb, base_pose.pose)

		self.pauseCommand()

		if not resp.success:
				rospy.logerr("MOVE_TO_POSE_INTERMEDIATE FAILED")
		#resp = self.move_arm(OPEN_GRIPPER, PoseStamped())
		resp = self.move_arm(1, self.limb, Pose())

		self.pauseCommand()
		#resp = self.move_arm(RESET_ARM, PoseStamped())
		base_pose.pose.position.z += .1
		resp = self.move_arm(3, self.limb, base_pose.pose)

	def interActionDelay(self, delay) : #if user tells command to quit, then you don't want delays to stack
	 	if self.command_state == self.RUNNING :
			rospy.sleep(delay)

	
	def run_mott_sequence(self) :

		
		self.grabObject(self.object_pose)
		
		self.interActionDelay(1)

		self.releaseObject(self.target_pose)

		if self.command_state == self.RUNNING :
			self.publishResponse("finished mott") #string must contain finished
		elif self.command_state == self.PAUSING :
			self.publishResponse("finished mott while pausing!?!?") 
		elif self.command_state == self.CANCELLED :
			self.publishResponse("quitting on user command") 

				

	def MottCallback(self, data) :

		self.command_state = self.RUNNING

		if data.object_pose_topic != "" :
			self.object_pose.subscribe(data.object_pose_topic)

		if data.target_pose_topic != "" :
			self.target_pose.subscribe(data.target_pose_topic)
		
		if (data.object_pose) :
			self.object_pose.set_pose(data.object_pose)

		if (data.target_pose) :
			self.target_pose.set_pose(data.target_pose)

		if data.command == "mott" :
			self.run_mott_sequence()


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


	def createServiceProxy(service,srv_type,limb):
		name = ""
		if limb == "":
			name = "/%s" % (service)
		else:
			name = "/%s/%s" % (limb,service)
		rospy.wait_for_service(name)
		rospy.loginfo("Initialized service proxy for %s" % name)
		return rospy.ServiceProxy(name, srv_type)


	def __init__(self):
		limb = "left"#TODO change to param or sumthing

		self.robot_name = "baxter_"+limb
		rospy.init_node('%s_nav_manip_controller'%self.robot_name)

		#self.robot_pose = DynamicPose()
		#self.robot_pose.subscribe("/robot_pose") #TODO change to end effector position??

		self.object_pose = DynamicPose()
		self.target_pose = DynamicPose()

		self.RUNNING = 0
		self.PAUSING = 1
		self.CANCELLED = 2
		self.command_state = 0 #keeps track of running, pausing, cancelled

		#self.FIXED_FRAME = "odom"
		self.BASE_FAME = "base_link"#TODO look into

		robot = "baxter_" + limb
		self.response_pub = rospy.Publisher("/"+robot+"_mott_response", String)
		self.baxter_cmd_pub = rospy.Publisher("/"+robot+"_cmd", Int32)

		rospy.Subscriber("/"+robot+"_mott", Mott, self.MottCallback, queue_size = 1)
		rospy.Subscriber("/"+robot+"_mott_command", String, self.MottCommandCallback, queue_size = 1)

		#self.move_arm = createServiceProxy("move/arm", MoveRobot, self.robot_name) TODO 
		self.move_arm = createServiceProxy("baxter/move/arm", MoveRobot, self.robot_name)

		self.test_pose_pub = rospy.Publisher('/test_obj_pose', PoseStamped)

		self.tfl = tf.TransformListener()

		rospy.spin()

if __name__ == "__main__":
	Nav_Manip_Controller()