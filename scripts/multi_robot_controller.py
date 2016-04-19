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

class Workspace:
        def __init__(self, p1, p2):
            self.p1 = p1
            self.p2 = p2

        def between(x, b1, b2):
            bounds = sorted([b1,b2])
            return b1 <= x and x <= b2

        def inside_workspace(self, point):
            # returns true if the point is inside the robot's workspace, otherwise false
            inx = between(point.x, self.p1.x, self.p2.x)
            iny = between(point.y, self.p1.y, self.p2.y)
            inz = between(point.z, self.p1.z, self.p2.z)
            return inx and iny and inz

class Robot:
    def __init__(self, name, color, workspace):
        self.name = name
        self.color = color
        self.workspace = workspace

    

    

# class HP:
#     def __init__(self, name, color, workspace):
#         self.transform = TransformStamped()

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
        # for r in self.robots:
        #     self.color_map[r.name] = r.color

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

    def init_handoff_zones(self):
        # xyz point and width, height
        pass



    def init_robots(self):
        self.robots = []
        # define name, color, and workspace
        baxter = Robot("baxter", 1.0, {
            "reference_frame" : "base",
            "x1" : -0.5,
            "y1" : -0.5,
            "z1" : -0.5,

            "x2" : 0.5,
            "y2" : 0.5,
            "z2" : 0.5
        })
        self.robots.append(baxter)

        height = 0.3
        gatlin = Robot("gatlin", 0.6, {
            "reference_frame" : "base_link",
            "x1" : -5.0,
            "y1" : -5.0,
            "z1" : 0.0,

            "x2" : 5.0,
            "y2" : 5.0,
            "z2" : height
        })
        self.robots.append(gatlin)

        youbot = Robot("youbot", 0.6, {
            "reference_frame" : "base_link",
            "x1" : -5.0,
            "y1" : -5.0,
            "z1" : 0.0,

            "x2" : 5.0,
            "y2" : 5.0,
            "z2" : height
        })
        self.robots.append(youbot)

        height = 0.2
        modbot = Robot("modbot", 0.6, {
            "reference_frame" : "base_link",
            "x1" : -5.0,
            "y1" : -5.0,
            "z1" : 0.0,

            "x2" : 5.0,
            "y2" : 5.0,
            "z2" : height
        })
        self.robots.append(modbot)

    def generate_workspace_connectivity_graph(self):
        # given a pose determine if it is in the robots workspace
        # ex. self.inside_workspace("baxter", xyz_point)
        # if yes then add an edge
        self.wcg = WorkspaceConnectivityGraph()


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

    def getTransform(self, parent, child):
        try:
            # self.tfl.waitForTransform(child, parent, rospy.Time(0), rospy.Duration(.5))
            (T,R) = self.tfl.lookupTransform(parent, child, rospy.Time(0))
        except:
            rospy.logerr("no transform %s -> %s" % (parent, child))
            return None

        return (T,R)

    def getDist(self, parent, child):
        (T,R) = self.getTransform(parent, child)
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
