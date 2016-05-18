#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
import tf
from std_msgs.msg import String, Int8MultiArray, Float32MultiArray, Time, Header, Int32
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import *
from nav_msgs.srv import *
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatusArray
from config import *

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

#wiki.ros.org/gmapping
#wiki.ros.org/move_base

class GmapperConverter:

	def __init__(self):
		rospy.init_node('GmapperConverter', anonymous=True)
		print "initializing GmapperConverter"

		self.LastGoalID = None

		#self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.imagecallback, queue_size=1)
		#self.gmap_sub = rospy.Subscriber("/map", OccupancyGrid, self.mapcallback, queue_size=1)
		self.gmap_pub = rospy.Publisher("/gatlin/gmap_array", Int8MultiArray)
		#self.gmapinfo_pub = rospy.Publisher("/gatlin/gmap_info", Float32MultiArray)
		#self.time_pub = rospy.Publisher( "/gatlin/time", Time)

		self.transform_listener = tf.TransformListener()
		self.map_pub = rospy.Publisher("/map_pose" , PoseStamped)

		self.robot_pose_sub = rospy.Subscriber("/robot_pose_ekf/odom_combined_throttled", PoseWithCovarianceStamped, self.robotposecallback, queue_size = 1)
		self.robot_pose_pub = rospy.Publisher("/robot_pose", PoseStamped)
		self.robot_pose_stamped = PoseStamped() 
		
		self.goal_pose_stamped_pub = rospy.Publisher("/goal_pose_stamped", PoseStamped)		
		self.move_to_goal_sub = rospy.Subscriber("/move_to_goal", PoseStamped, self.goalcallback, queue_size = 1)
		self.next_path_pub = rospy.Publisher("/gatlin_path", Path)
		self.move_to_goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal)
		self.move_to_goal_count = 0

		self.cancel_goal_sub = rospy.Subscriber("/gatlin_cmd", Int32, self.cancelcallback, queue_size=1)
		self.cancel_move_pub = rospy.Publisher("/move_base/cancel", GoalID) #(actionlib_msgs/GoalID)

		#move_base/feedback (move_base_msgs/MoveBaseActionFeedback)

		self.status_sub = rospy.Subscriber("/move_base/status", GoalStatusArray, self.statuscallback, queue_size = 1)

		# self.get_path_to_goal_srv = createServiceProxy("/make_plan", GetPlan)

		# self.lastnppose = np.array([-10000.0, -100000, -10000])

		#tell the action client that we want to spin a thread by default
		self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		rospy.loginfo("wait for the action server to come up")
		#allow up to 5 seconds for the action server to come up
		self.move_base.wait_for_server(rospy.Duration(5))

		print "done init"

		# self.GoForwardAvoid()

		rospy.spin()
		
	def robotposecallback(self, data) :
		ps = PoseStamped()
		ps.header = data.header
		ps.pose = data.pose.pose
		self.robot_pose_stamped = ps
		self.robot_pose_pub.publish(ps)

	#receives a pose and publishes a goal for gmapper
	def goalcallback(self, data):



		#we'll send a goal to the robot to move 3 meters forward
		goal = MoveBaseGoal()
		# goal.target_pose.header.frame_id = 'base_link'
		# goal.target_pose.header.stamp = rospy.Time.now()
		# goal.target_pose.pose.position.x = 0.2
		# goal.target_pose.pose.orientation.w = 1.0
		goal.target_pose = data
		#start moving
		rospy.logerr(goal)
		# self.move_base.send_goal(goal)




		#TODO check status of route, if on another route, cancel it first
		# self.cancelMovement()


		t = rospy.Time.now()

		#draws the path 
		req = GetPlanRequest()
		req.start = self.robot_pose_stamped
		req.start.header.stamp = t
		req.goal = data
		req.goal.header.stamp = t
		req.tolerance = .02

		# return_path = self.get_path_to_goal_srv(req)
		# self.next_path_pub.publish(return_path)

		self.move_to_goal_count += 1
		g = MoveBaseActionGoal();
		t = rospy.Time.now()
		g.header = Header(self.move_to_goal_count, t, "map")
		g.goal_id.stamp = t
		g.goal_id.id = "movement_num:"+str(self.move_to_goal_count)
		g.goal.target_pose = data

		# newnppose = vector3_to_numpy(data.pose.position)
		# if np.linalg.norm(newnppose - self.lastnppose) >.05 :

		# self.goal_pose_stamped_pub.publish(data)
		self.move_to_goal_pub.publish(g)
		rospy.logerr("published move_base goal")

		# self.lastnp_pose = newnppose

	def cancelcallback(self, msg) :
		print "command in"
		if msg.data == 9:
			print "Cancel callback came in"
			self.cancelMovement()

	def cancelMovement(self) :
		print "Cancelling current action"
		self.move_base.cancel_goal()

		# if self.LastGoalID != None:
		# 	self.cancel_move_pub.publish(self.LastGoalID);
	
	def statuscallback(self, data) :
		#received status callback
		l = len(data.status_list)
		#last is data.status_list[l-1].goal_id
		for i in xrange(0, l) :
			#print data.status_list[i].status
			if data.status_list[i].status == 1 :#this is the active one
				#print "Active GOAL:"
				#print data.status_list[i].goal_id
				self.LastGoalID = data.status_list[i].goal_id
				return
		self.LastGoalID = None

	def GoForwardAvoid(self):
		#what to do if shut down (e.g. ctrl + C or failure)
		# rospy.on_shutdown(self.shutdown)

		
		

		#we'll send a goal to the robot to move 3 meters forward
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'base_link'
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = 0.2
		goal.target_pose.pose.orientation.w = 1.0

		#start moving
		self.move_base.send_goal(goal)

		rospy.sleep(2)
		self.cancelMovement()

		#allow TurtleBot up to 60 seconds to complete task
		success = self.move_base.wait_for_result(rospy.Duration(60)) 


		if not success:
				self.move_base.cancel_goal()
				rospy.loginfo("The base failed to move forward 3 meters for some reason")
		else:
		# We made it!
			state = self.move_base.get_state()
			if state == GoalStatus.SUCCEEDED:
				rospy.loginfo("Hooray, the base moved 3 meters forward")

if __name__ == '__main__':
	GmapperConverter()
