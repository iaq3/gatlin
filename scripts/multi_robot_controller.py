#!/usr/bin/env python

from config import *
import numpy as np
import rospy
from geometry_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *
import networkx as nx
import matplotlib.pyplot as plt
import tf

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
            pass

    def init_handoff_zones(self):
        # xyz point and width, height
        pass

    def inside_workspace(self, robot, point):
        # returns true if the point is inside the robot's workspace
        pass

    def generate_workspace_connectivity_graph(self):
        # given a pose determine if it is in the robots workspace
        # ex. self.inside_workspace("baxter", xyz_point)
        # if yes then add an edge
        pass

class WorkspaceConnectivityGraph:
    def __init__(self):
        rospy.init_node('WorkspaceConnectivityGraph')
        self.tfl = tf.TransformListener()

        self.color_map = {
            'baxter': 1.0,
            'gatlin': 0.6,
            'youbot': 0.6,
            'modbot': 0.6,
        }

        self.G = nx.DiGraph()

        self.add_hp('gatlin', 'hp1', .7)
        self.add_hp('gatlin', 'hp2', .2)
        self.add_obj('gatlin', 'obj1', .6)

        self.add_hp('baxter', 'hp1', .3)
        self.add_target('baxter', 'target1', .2)
        self.add_target('baxter', 'target2', .25)
        self.add_target('baxter', 'target3', .3)

        self.add_hp('youbot', 'hp1', .25)
        self.add_hp('youbot', 'hp2', .3)
        self.add_obj('youbot', 'obj2', .5)
        self.add_obj('youbot', 'obj3', .3)

        self.draw()
        self.show()

        print self.find_shortest_path('obj1', 'target1')
        print self.find_shortest_path('obj2', 'target2')
        print self.find_shortest_path('obj3', 'target3')

        # self.update_distances()
        # self.draw()
        # self.show()

    def add_hp(self, robot, hp, robot_to_hp):
        self.color_map[hp] = 0.25
        # robot_to_hp = self.getDist(robot, hp)
        self.G.add_edge(robot, hp, distance=robot_to_hp)
        self.G.add_edge(hp, robot, distance=robot_to_hp)

    def add_obj(self, robot, obj, robot_to_obj):
        self.color_map[obj] = 0.5
        # robot_to_obj = self.getDist(robot, obj)
        self.G.add_edge(obj, robot, distance=robot_to_obj)

    def add_target(self, robot, target, robot_to_target):
        self.color_map[target] = 0.0
        # robot_to_target = self.getDist(robot, target)
        self.G.add_edge(robot, target, distance=robot_to_target)

    def update_distances(self):
        edges,weights = zip(*nx.get_edge_attributes(self.G,'distance').items())
        for u,v in edges:
            newdist = self.getDist(u,v)
            self.G.add_edge(u,v, distance=newdist)

    def getDist(self, parent, child):
        try:
            # self.tfl.waitForTransform(child, parent, rospy.Time(0), rospy.Duration(.5))
            (T,R) = self.tfl.lookupTransform(parent, child, rospy.Time(0))
        except:
            rospy.logerr("no transform %s -> %s" % (parent, child))
            return 100.0
        T = np.array(T)
        return np.linalg.norm(T)

    def draw(self):
        node_colors = [self.color_map.get(node, 0.0) for node in self.G.nodes()]
        edges,weights = zip(*nx.get_edge_attributes(self.G,'distance').items())

        nx.draw(self.G, font_color='m', with_labels=True, node_color=node_colors, edgelist=edges, edge_color=weights, width=5.0, edge_cmap=plt.cm.YlOrRd)

        plt.draw()

    def show(self):
        plt.show()

    def find_shortest_path(self, v1, v2):
        try:
            print nx.dijkstra_path_length(self.G, v1, v2, 'distance')
            return nx.dijkstra_path(self.G, v1, v2, 'distance')
        except Exception as e:
            print e
            # rospy.logerr(e)
            return None


if __name__ == '__main__':
    wcg = WorkspaceConnectivityGraph()
    # mrc = MRC()
