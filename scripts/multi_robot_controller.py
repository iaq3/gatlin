#!/usr/bin/env python

from config import *
import numpy as np
import rospy
from geometry_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *

class MRC:
    def __init__(self):
        rospy.init_node('multi_robot_controller')

        rospy.Subscriber("/mr_command_req", CommandListRequest, self.mr_command_req_callback)

        # publishers for all service requests
        # subscribers for all service responses

        rospy.spin()

    def mr_command_req_callback(self, cmd_req):
        rospy.logerr(cmd_req)
        # recieve commands
        for cmd in cmd_req.commands:
            # generate connectivity graph
                # given a pose determine if it is in the robots wksp
                # if yes then add an edge

            # find path from object to target in connectivity graph
            # generate actions based on optimal path
            # use dependencies to build action graph
            # put actions in queue

    def init_handoff_zones(self):
        # xyz point and width, height
        pass

    def inside_workspace(self, robot, point):
        # returns true if the point is inside the robot's workspace

    def generate_workspace_connectivity_graph(self):
        # given a pose determine if it is in the robots workspace
        # ex. self.inside_workspace("baxter", xyz_point)
        # if yes then add an edge
        pass

class WeightedGraph:
    def __init__(self):
        # lists of nodes edges
        pass

    def add_node(self, v1):
        pass

    def add_edge(self, v1, v2, cost):
        pass

    def find_shortest_path(self, v1, v2):
        pass



if __name__ == '__main__':
    mrc = MRC()    
