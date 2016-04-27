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

        self.init_robots()
        self.wcg = WorkspaceConnectivityGraph(self.robots)

        rospy.spin()

    def publish_mott(self, robot, mott):
        self.mott_pubs[robot].publish(mott)
        rospy.loginfo("PUBLISHED MOTT")
        rospy.loginfo(robot)
        rospy.loginfo(mott)

    def init_robots(self):
        self.robots = []
        # define name, color, and workspace
        baxter_left = Robot("baxter_left",  1.0,
            Workspace(
                Point(-0.5,-0.5,-0.5),
                Point(0.5,0.0,0.5),
                "base"
            )
        )
        self.robots.append(baxter)

        baxter_right = Robot("baxter_right",  1.0,
            Workspace(
                Point(-0.5,0.0,-0.5),
                Point(0.5,0.5,0.5),
                "base"
            )
        )
        self.robots.append(baxter)

        height = 0.3
        gatlin = Robot("gatlin", 0.6,
            Workspace(
                Point(-5.0,-5.0,0.0),
                Point(5.0,5.0,height),
                "base_link"
            )
        )
        self.robots.append(gatlin)

        youbot = Robot("youbot", 0.6, 
            Workspace(
                Point(-5.0,-5.0,0.0),
                Point(5.0,5.0,height),
                "base_link"
            )
        )
        self.robots.append(youbot)

        height = 0.2
        modbot = Robot("modbot", 0.6,
            Workspace(
                Point(-5.0,-5.0,0.0),
                Point(5.0,5.0,height),
                "base_link"
            )
        )
        self.robots.append(modbot)

    def mr_command_req_callback(self, cmd_req):
        rospy.logerr(cmd_req)

        # json.loads('["foo", {"bar":["baz", null, 1.0, 2]}]')

        
        

        # recieve commands
        for cmd in cmd_req.commands:
            # cmd["id"]
            # cmd["action"]
            # cmd["target_pose"]
            # cmd["obj_pose"]
            # cmd["obj_topic"]

            object_pose = cmd["obj_pose"]
            target_pose = cmd["target_pose"]

            # generate connectivity graph
            self.wcg.add_target(target)

                # given a pose determine if it is in the robots wksp
                # if yes then add an edge

            # find path from object to target in connectivity graph
            # generate actions based on optimal path
            # use dependencies to build action graph
            # put actions in queue

class DynamicPose:
    def __init__(self):
        # self.FIXED_FRAME = "global_map"
        self.FIXED_FRAME = "base"
        self.tfl = tf.TransformListener()
        self.ps = PoseStamped()
        self.last_update = 0
        self.topic = ""

    def subscribe(self, topic, dps):
        self.reset()
        dps.append((topic, self))
        self.topic = topic

    def reset(self, dps):
        self.ps = PoseStamped()
        self.last_update = 0
        try:
            dps.remove((self.topic, self))
            rospy.logerr("dp found")
        except:
            rospy.logerr("dp not found")
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

class Workspace:
        def __init__(self, rf, p1, p2):
            self.reference_frame = rf
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
        # publishers for all service requests
        self.mott_pub = rospy.Publisher("/%s_mott" % name, Mott, queue_size = 1)
        self.mott_command_pub = rospy.Publisher("/%s_mott_command" % name, String, queue_size = 1)

        # subscribers for all service responses
        self.response_sub = rospy.Subscriber("/%s_mott_response" % name, String, queue_size = 1)

    def mott_response_callback(self, resp):
        rospy.logerr(resp)


# class HP:
#     def __init__(self, name, color, workspace):
#         self.transform = TransformStamped()

class WorkspaceConnectivityGraph:
    def __init__(self, robots):
        rospy.init_node('WorkspaceConnectivityGraph')
        self.tfl = tf.TransformListener()

        self.robots = robots

        # self.color_map = {
        #     'baxter': 1.0,
        #     'gatlin': 0.6,
        #     'youbot': 0.6,
        #     'modbot': 0.6,
        # }
        for r in self.robots:
            self.color_map[r.name] = r.color

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



    

    def generate_workspace_connectivity_graph(self):
        # given a pose determine if it is in the robots workspace
        # ex. self.inside_workspace("baxter", xyz_point)
        # if yes then add an edge
        # self.wcg = WorkspaceConnectivityGraph()


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
    # wcg = WorkspaceConnectivityGraph()
    mrc = MRC()
