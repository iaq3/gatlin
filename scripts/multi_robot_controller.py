#!/usr/bin/env python

from config import *
import numpy as np
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *
import networkx as nx
# sudo apt-get install python-networkx
import matplotlib.pyplot as plt
import tf
from rospy_message_converter import json_message_converter
# sudo apt-get install ros-indigo-rospy-message-converter

class MRC:
    def __init__(self):
        rospy.init_node('multi_robot_controller')

        rospy.Subscriber("/mr_command_req", CommandRequestList, self.mr_command_req_callback, queue_size = 1)

        self.init_robots()

        self.display_workspaces()
        
        self.wcg = WorkspaceConnectivityGraph(self.robots)

        m = Mott()
        m.command = "mott"
        m.object_pose_topic = "ar_7"
        m.target_pose_topic = "target_1"

        # m.object_pose = PoseStamped()

        m.target_pose = PoseStamped()
        m.target_pose.header.frame_id = "base"
        m.target_pose.header.stamp = rospy.Time.now()
        m.target_pose.pose.position = Point(.5,.5,.5)
        m.target_pose.pose.orientation = Quaternion(0,0,0,1)
        mott_json = json_message_converter.convert_ros_message_to_json(m)

        # test CommandRequestList
        crl = CommandRequestList()
        cr = CommandRequest()
        cr.id = 1
        cr.action = "mott"
        cr.args = mott_json
        crl.commands.append(cr)

        self.mr_command_req_callback(crl)

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
                Point( 0.80,-0.10,-0.60),
                Point(-0.30, 0.80, 0.70),
                "base"
                # "global_map"
            )
        )
        self.robots.append(baxter_left)

        baxter_right = Robot("baxter_right",  1.0,
            Workspace(
                Point( 0.80,-0.80,-0.60),
                Point(-0.30, 0.10, 0.70),
                "base"
            )
        )
        self.robots.append(baxter_right)

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

    def display_workspaces(self):
        self.workspace_pub = rospy.Publisher("/workspace_markers", Marker, queue_size = 1)
        
        rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        for x in range(0,20):
            for i in range(0,len(self.robots)):
                self.display_workspace(self.robots[i].workspace, i)
            rate.sleep()

    def display_workspace(self, workspace, i):
        center, dimensions = workspace.getCenterDimensions()

        marker = Marker()
        marker.header.frame_id = workspace.reference_frame
        marker.header.stamp = rospy.Time.now()
        marker.id = i
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = dimensions[0]
        marker.scale.y = dimensions[1]
        marker.scale.z = dimensions[2]
        marker.color.a = 0.5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        self.workspace_pub.publish(marker)
        # rospy.logerr(marker)

    def mr_command_req_callback(self, cmd_req):
        rospy.logerr(cmd_req)

        # def test_ros_message_with_string(self):
        # from std_msgs.msg import String
        # expected_json = '{"data": "Hello"}'
        # message = String(data = 'Hello')
        # returned_json = json_message_converter.convert_ros_message_to_json(message)
        # self.assertEqual(returned_json, expected_json)

        # def test_json_with_string(self):
        # from std_msgs.msg import String
        # expected_message = String(data = 'Hello')
        # json_str = '{"data": "Hello"}'
        # message = json_message_converter.convert_json_to_ros_message('std_msgs/String', json_str)
        # self.assertEqual(message, expected_message)

        # recieve commands
        for cmd in cmd_req.commands:
            mott = json_message_converter.convert_json_to_ros_message('gatlin/Mott', cmd.args)
            rospy.logerr(mott)
            
            # cmd.id
            # cmd.action

            # mott.command
            # mott.object_pose_topic
            # mott.target_pose_topic
            # mott.object_pose
            # mott.target_pose

            # generate connectivity graph
            self.wcg.add_obj(mott.object_pose_topic)
            self.wcg.add_target(mott.target_pose_topic)

                # given a pose determine if it is in the robots wksp
                # if yes then add an edge

            # find path from object to target in connectivity graph
            # generate actions based on optimal path
            # use dependencies to build action graph
            # put actions in queue


class Workspace:
        def __init__(self, p1, p2, rf):
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

        def getCenterDimensions(self):
            p1 = vector3_to_numpy(self.p1)
            p2 = vector3_to_numpy(self.p2)
            center = (p1+p2)/2
            dimensions = abs(p1-center)*2
            return center, dimensions


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
        # rospy.init_node('WorkspaceConnectivityGraph')
        self.tfl = tf.TransformListener()

        self.robots = robots

        self.color_map = {}
        for r in self.robots:
            self.color_map[r.name] = r.color

        self.G = nx.DiGraph()

        # self.fixed_frame = "global_map"
        self.fixed_frame = "base"
        self.dm = DynamicManager()
        self.dm.add_ol_sub("/server/ar_marker_list")

        # self.add_hp('gatlin', 'hp1', .7)
        # self.add_hp('gatlin', 'hp2', .2)
        # self.add_obj('gatlin', 'obj1', .6)

        # self.add_hp('baxter', 'hp1', .3)
        # self.add_target('baxter', 'target1', .2)
        # self.add_target('baxter', 'target2', .25)
        # self.add_target('baxter', 'target3', .3)

        # self.add_hp('youbot', 'hp1', .25)
        # self.add_hp('youbot', 'hp2', .3)
        # self.add_obj('youbot', 'obj2', .5)
        # self.add_obj('youbot', 'obj3', .3)

        self.add_obj('baxter_right', 'ar_7', .6)
        self.add_hp('baxter_right', 'hp1', .3)
        self.add_hp('baxter_left', 'hp1', .3)
        self.add_target('baxter_left', 'target1', .2)
        print self.find_shortest_path('ar_7', 'target1')

        self.draw()
        self.show()

        # print self.find_shortest_path('obj1', 'target1')
        # print self.find_shortest_path('obj2', 'target2')
        # print self.find_shortest_path('obj3', 'target3')

        # self.update_distances()
        # self.draw()
        # self.show()

    def init_handoff_zones(self):
        # xyz point and width, length
        table_z = -.230

        hp1 = Workspace(
            Point(0.70,-0.10, table_z),
            Point(0.50, 0.10, table_z+.01),
            "base"
        )

    def generate_workspace_connectivity_graph(self):
        # given a pose determine if it is in the robots workspace
        # ex. self.inside_workspace("baxter", xyz_point)
        # if yes then add an edge
        # self.wcg = WorkspaceConnectivityGraph()
        pass


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

    def add_obj(self, obj):
        # create_dp
        object_pose = self.dm.create_dp(self.fixed_frame)
        object_pose.subscribe_name(obj)

        self.objects.append(object_pose)

        for robot in self.robots:
            # in workspace?
            inside = robot.workspace.inside_workspace(object_pose.ps.pose.position)
            if inside:
                robot_to_obj = self.getDist(robot, obj)
                self.add_obj_edge(robot.name, obj, robot_to_obj)

    def add_obj_edge(self, robot, obj, robot_to_obj):
        self.color_map[obj] = 0.5
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
