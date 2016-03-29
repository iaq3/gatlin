#!/usr/bin/env python
import rospy
from std_msgs.msg import *
from copy import deepcopy
from config import *

class TestService:
	def __init__(self):
		rospy.init_node('TestService')

		srv_name = "/test"
		self.req_sub = rospy.Subscriber("%s/request" % srv_name, String, self.handle_req, queue_size = 1)
		self.resp_pub = rospy.Publisher("%s/response" % srv_name, String, queue_size = 1)

		rospy.spin()

	def handle_req(self, req):
		# rospy.logerr(req)
		self.resp_pub.publish("Done")

if __name__ == "__main__":
	TestService()