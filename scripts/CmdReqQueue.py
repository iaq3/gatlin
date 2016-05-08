#!/usr/bin/env python
# Names: Zach Vinegar, Isaac Qureshi
import rospy, tf, time
import numpy as np
from geometry_msgs.msg import *
from tf.transformations import *
from copy import deepcopy
from gatlin.msg import *
from rospy_message_converter import json_message_converter

class CommandReqQueue:
	def __init__(self):

		self.cmd_reqs = {} #current action nodes in the system
		self.robot_cmds = {} #action nodes for each robot

	def add_robot(self, robot) :
		self.robot_cmds[robot] = []

	def add_command_req(self, cmd_req, robot):
		new_cmd_node = cmd_reqNode(cmd_req)
		self.cmd_reqs[cmd_req.id] = new_cmd_node
		self.robot_cmds[robot].append(new_cmd_node)

	#do cmd_req1 first, then after do cmd_req2
	def add_dependency(self, cmd_id1, cmd_id2):
		self.cmd_reqs[cmd_id1].add_postreq(self.cmd_reqs[cmd_id2]) #could be just id of it too
		self.cmd_reqs[cmd_id2].add_prereq(self.cmd_reqs[cmd_id1])


	#finds the most important action for a robot, sets it to running, and returns it
	def request_command(self, robot) :
		max_postreqs = -1
		best_cmd_reqNode = None
		global MATCHED_RUNNING, AVAILABLE_WAITING
		for v in self.robot_cmds[robot] :
			if v.post_req_count > max_postreqs and v.state == AVAILABLE_WAITING :
				max_postreqs = v.post_req_count
				best_cmd_reqNode = v
			if v.state == MATCHED_RUNNING :
				rospy.loginfo("Command is already running on %s." % robot)
		if best_cmd_reqNode != None :
			rospy.loginfo("%s given command %s." % (robot, best_cmd_reqNode.cmd_req.id))
			best_cmd_reqNode.state = MATCHED_RUNNING
			return best_cmd_reqNode.cmd_req

		rospy.loginfo("No commands availible for %s." % robot)
		return None

	#alerts all postreq actions that prereq is finished
	#deletes this action node from the 2 data structures
	#returns the best action for this robot now
	def robot_finished(self, robot, cmd_req) :
		self.cmd_reqs[cmd_req.id].action_finished()		

		self.robot_cmds[robot].remove(self.cmd_reqs[cmd_req.id])
		
		del self.cmd_reqs[cmd_req.id]

		# return self.request_command(robot)



	def robot_quit(self, robot, cmd_req) :
		self.cmd_reqs[cmd_req.id].action_quit()
		return self.request_command(robot)

	def can_execute(self, cmd_req):
		# if all prereqs are finished return True
		global AVAILABLE_WAITING
		return self.cmd_reqs[cmd_req.id].state == AVAILABLE_WAITING

#PREREQ_WAITING 	-> available_waiting (for robot) 	-> matched_running 	-> finished (post_reqs -1 )
#

PREREQ_WAITING = 1
AVAILABLE_WAITING = 2
MATCHED_RUNNING = 3
FINISHED = 4

class cmd_reqNode :
	
	def __init__(self, cmd_r) :

		self.cmd_req = cmd_r

		self.prereqs = []
		self.postreqs = []

		self.pre_req_count = 0
		self.post_req_count = 0

		#state of action node
		global AVAILABLE_WAITING
		self.state = AVAILABLE_WAITING

	def add_prereq(self, pre_req) :
		self.prereqs.append(pre_req)
		self.pre_req_count += 1

		global PREREQ_WAITING
		self.state = PREREQ_WAITING

	def add_postreq(self, post_req) :
		self.postreqs.append(post_req)
		self.post_req_count += 1

	def prereq_complete(self) :
		self.pre_req_count -= 1
		global AVAILABLE_WAITING
		if self.pre_req_count == 0 :
			self.state = AVAILABLE_WAITING

	def robot_starting_action(self) :
		global MATCHED_RUNNING
		self.state = MATCHED_RUNNING


	def action_quit(self) :
		if self.post_req_count > 1 :
			self.post_req_count -= 1

		global AVAILABLE_WAITING
		self.state = AVAILABLE_WAITING

	#alerts all children of this action that they have one less pre
	def action_finished(self) :
		global FINISHED
		self.state = FINISHED
		for e in self.postreqs :
			e.prereq_complete()


if __name__ == '__main__':

	aq = CommandReqQueue()
	aq.add_robot("baxter_left")
	aq.add_robot("baxter_right")

	m1 = Mott()
	m1.command = "mott"
	m1.object_pose_topic = "ar_7"
	m1.object_pose = PoseStamped()
	m1.target_pose_topic = "hp_1"
	m1.target_pose = PoseStamped()
	rospy.logerr(m1)


	m2 = Mott()
	m2.command = "mott"
	m2.object_pose_topic = "ar_7"
	m2.object_pose = PoseStamped()
	m2.target_pose_topic = "target_1"
	m2.target_pose = PoseStamped()
	rospy.logerr(m2)


	m1_json = json_message_converter.convert_ros_message_to_json(m1)
	m2_json = json_message_converter.convert_ros_message_to_json(m2)

	# test CommandRequestList
	crl = CommandRequestList()

	cr1 = CommandRequest()
	cr1.id = 1
	cr1.action = "mott"
	cr1.args = m1_json
	crl.commands.append(cr1)

	cr2 = CommandRequest()
	cr2.id = 2
	cr2.action = "mott"
	cr2.args = m2_json
	crl.commands.append(cr2)



	cmd_req = aq.request_command("baxter_left")
	if cmd_req == None :
		print "passed test 0"
	else :
		print "failed test 0"

	aq.add_command_req(cr1, "baxter_right")
	aq.add_command_req(cr2, "baxter_left")

	aq.add_dependency(cr1.id, cr2.id)

	cmd_req = aq.request_command("baxter_left")
	if cmd_req == None :
		print "passed test 1"
	else :
		print "failed test 1"
	# returns None

	cmd_req = aq.request_command("baxter_right")
	print cmd_req
	# returns cr1



