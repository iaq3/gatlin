#!/usr/bin/env python
# Names: Zach Vinegar, Isaac Qureshi
import rospy, tf, time
import numpy as np
from geometry_msgs.msg import *
from tf.transformations import *
from copy import deepcopy
from gatlin.msg import *

class ActionQueue:
	def __init__(self):
		self.actions = []

	def add_action(self, action):

	def add_dependency(self, action1, action2):

	def can_execute(self, action):
		# if all prereqs are finished return True
		return True