#!/usr/bin/env python

from config import *
from Dynamic import *
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

        table_z = -.230
        m.object_pose.header.frame_id = "base"
        m.object_pose.header.stamp = rospy.Time.now()
        m.object_pose.pose.position = Point(.6,-.2, table_z)
        m.object_pose.pose.orientation = Quaternion(0,0,0,1)
        
        m.target_pose.header.frame_id = "base"
        m.target_pose.header.stamp = rospy.Time.now()
        m.target_pose.pose.position = Point(.6,.3, table_z)
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

    def find_robot(self, name):
        for r in self.robots:
            if r.name == name:
                return r

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

        # height = 0.3
        # gatlin = Robot("gatlin", 0.6,
        #     Workspace(
        #         Point(-5.0,-5.0,0.0),
        #         Point(5.0,5.0,height),
        #         "base_link"
        #     )
        # )
        # self.robots.append(gatlin)

        # youbot = Robot("youbot", 0.6, 
        #     Workspace(
        #         Point(-5.0,-5.0,0.0),
        #         Point(5.0,5.0,height),
        #         "base_link"
        #     )
        # )
        # self.robots.append(youbot)

        # height = 0.2
        # modbot = Robot("modbot", 0.6,
        #     Workspace(
        #         Point(-5.0,-5.0,0.0),
        #         Point(5.0,5.0,height),
        #         "base_link"
        #     )
        # )
        # self.robots.append(modbot)

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

        # recieve commands
        for cmd in cmd_req.commands:
            if cmd.action == "mott":
                mott = json_message_converter.convert_json_to_ros_message('gatlin/Mott', cmd.args)
                # rospy.logerr(mott)
                
                # cmd.id
                # cmd.action

                # mott.command
                # mott.object_pose_topic
                # mott.target_pose_topic
                # mott.object_pose
                # mott.target_pose

                # generate connectivity graph
                self.wcg.add_obj(mott.object_pose_topic, mott.object_pose)
                self.wcg.add_target(mott.target_pose_topic, mott.target_pose)

                path = self.wcg.find_shortest_path(mott.object_pose_topic, mott.target_pose_topic)
                rospy.logerr(path)

                self.wcg.draw()
                self.wcg.show()

                if path != None:
                    self.wcg.generate_actions(path)

                # find path from object to target in connectivity graph
                # generate actions based on optimal path
                # use dependencies to build action graph
                # put actions in queue


class Workspace:
        def __init__(self, p1, p2, rf):
            self.reference_frame = rf
            self.p1 = p1
            self.p2 = p2

        def between(self, x, b1, b2):
            bounds = sorted([b1,b2])
            return bounds[0] <= x and x <= bounds[1]

        def inside_workspace(self, point):
            # returns true if the point is inside the robot's workspace, otherwise false
            inx = self.between(point.x, self.p1.x, self.p2.x)
            iny = self.between(point.y, self.p1.y, self.p2.y)
            inz = self.between(point.z, self.p1.z, self.p2.z)
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

        self.objects = []
        self.targets = []
        self.hps = []

        self.init_handoff_zones()

        # self.update_distances()
        # self.draw()
        # self.show()

    def generate_actions(self, path):
        # find path from object to target in connectivity graph
        # generate actions based on optimal path
        # use dependencies to build action graph
        # put actions in queue
        num_robots = (len(path)-1)/2
        num_hp = (len(path)-3)/2
        if num_hp < 1: num_hp = 0

        object_name = path[0]
        object_dp = self.find_object(object_name)

        if object_dp == None:
            rospy.logerr("no dp for %s" % object_name)
            return

        for i in range(0, num_robots):
            robot_name = path[i*2+1]
            target_name = path[i*2+2]
            target_dp = self.find_target(target_name)

            if target_dp == None:
                rospy.logerr("no dp for %s" % target_name)
                continue

            m = Mott()
            m.command = "mott"
            m.object_pose_topic = object_name
            m.object_pose = object_dp.ps
            m.target_pose_topic = target_name
            m.target_pose = target_dp.ps
            rospy.logerr(m)

    def find_target(self, target_name):
        for target_dp in self.targets:
            if target_dp.get_name() == target_name:
                return target_dp

        for hp_dp in self.hps:
            if hp_dp.get_name() == target_name:
                return hp_dp

        return None

    def find_object(self, object_name):
        for object_dp in self.objects:
            if object_dp.get_name() == object_name:
                return object_dp
        return None

    def init_handoff_zones(self):
        # xyz point and width, length
        table_z = -.230

        hp_1 = Workspace(
            Point(0.70,-0.10, table_z),
            Point(0.50, 0.10, table_z+.01),
            "base"
        )
        self.add_hp("hp_1", hp_1)

    def add_hp(self, hp_name, hp_workspace):
        center, dimensions = hp_workspace.getCenterDimensions()
        init_hp_pose = PoseStamped()
        init_hp_pose.header.frame_id = hp_workspace.reference_frame
        init_hp_pose.header.stamp = rospy.Time.now()
        init_hp_pose.pose.position.x = center[0]
        init_hp_pose.pose.position.y = center[1]
        init_hp_pose.pose.position.z = center[2]
        # init_hp_pose.pose.orientation.w = 1
        # create dp
        hp_dp = self.dm.create_dp(self.fixed_frame)
        hp_dp.subscribe_name(hp_name)
        hp_dp.set_pose(init_hp_pose)

        self.hps.append(hp_dp)

        # given a obj determine if it is in the robots wksp
        for robot in self.robots:
            inside = robot.workspace.inside_workspace(hp_dp.ps.pose.position)
            # if inside then add an edge
            if inside:
                robot_to_hp = self.getDist(robot, hp_dp)
                # rospy.logerr(robot_to_hp)
                self.add_hp_edge(robot.name, hp_name, robot_to_hp)

    def add_hp_edge(self, robot, hp, robot_to_hp):
        self.color_map[hp] = 0.25
        # robot_to_hp = self.getDist(robot, hp)
        self.G.add_edge(robot, hp, distance=robot_to_hp)
        self.G.add_edge(hp, robot, distance=robot_to_hp)

    def add_obj(self, obj_topic, obj_pose):
        # create_dp
        object_dp = self.dm.create_dp(self.fixed_frame)
        object_dp.subscribe_name(obj_topic)
        if obj_pose.pose != Pose():
            object_dp.set_pose(obj_pose)
        else:
            rospy.logerr("obj_pose is identity")

        self.objects.append(object_dp)

        # given a obj determine if it is in the robots wksp
        for robot in self.robots:
            inside = robot.workspace.inside_workspace(object_dp.ps.pose.position)
            # if inside then add an edge
            if inside:
                robot_to_obj = self.getDist(robot, object_dp)
                # rospy.logerr(robot_to_obj)
                self.add_obj_edge(robot.name, obj_topic, robot_to_obj)

    def add_obj_edge(self, robot, obj_topic, robot_to_obj):
        self.color_map[obj_topic] = 0.5
        self.G.add_edge(obj_topic, robot, distance=robot_to_obj)

    def add_target(self, target_topic, init_target_pose):
        # create_dp
        target_dp = self.dm.create_dp(self.fixed_frame)
        target_dp.subscribe_name(target_topic)
        target_dp.set_pose(init_target_pose)

        self.targets.append(target_dp)

        # given a target determine if it is in the robots wksp
        for robot in self.robots:
            inside = robot.workspace.inside_workspace(target_dp.ps.pose.position)
            # if inside then add an edge
            if inside:
                robot_to_target = self.getDist(robot, target_dp)
                # rospy.logerr(robot_to_target)
                self.add_target_edge(robot.name, target_topic, robot_to_target)

    def add_target_edge(self, robot, target, robot_to_target):
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
            rospy.logerr("no transform %s -> %s" % (ps.header.frame_id, new_frame))
            return None

    # def getDist(self, parent, child):
    #     (T,R) = self.getTransform(parent, child)
    #     T = np.array(T)
    #     return np.linalg.norm(T)

    def getDist(self, robot, dp):
        new_pose = self.transform_pose(robot.name, dp.ps)
        T = vector3_to_numpy(new_pose.pose.position)
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
