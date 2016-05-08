#!/usr/bin/env python

from config import *
from Dynamic import *
from CmdReqQueue import *
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

        self.tfl = tf.TransformListener()

        self.crq = CommandReqQueue()
        self.crq.add_robot("baxter_left")
        self.crq.add_robot("baxter_right")
        self.crq.add_robot("gatlin")

        self.init_robots()
        
        self.wcg = WorkspaceConnectivityGraph(self.robots, self.crq, self.tfl)

        self.one_block_move()
        # self.two_block_stack()

        self.crq_locked = False

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if not self.crq_locked:
                for r in self.robots:
                    cmd = self.crq.request_command(r.name)
                    if cmd != None:
                        # rospy.logerr(cmd)
                        r.execute_command(cmd)
            self.display_workspaces()
            rate.sleep()

        rospy.spin()

    def one_block_move(self):
        m = Mott()
        m.command = "mott"
        m.object_pose_topic = "ar_5"
        m.target_pose_topic = "target_1"

        # m.object_pose = PoseStamped()

        table_z = -.230
        m.object_pose.header.frame_id = "baxter"
        # m.object_pose.header.stamp = rospy.Time.now()
        # m.object_pose.pose.position = Point(.6,-.5, table_z)
        # m.object_pose.pose.orientation = Quaternion(0,1,0,0)
        
        hp2_z = -0.548
        m.target_pose.header.frame_id = "baxter"
        m.target_pose.header.stamp = rospy.Time.now()
        m.target_pose.pose.position = Point(.6,-.5, table_z)
        # m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
        m.target_pose.pose.orientation = Quaternion(0,1,0,0)
        # rospy.logerr(m)

        # m.object_pose.header.frame_id = "gatlin"
        # m.object_pose.header.stamp = rospy.Time.now()
        # m.object_pose.pose.position = Point(0.21,-0.37, 0.03)
        # m.object_pose.pose.orientation = Quaternion(0,1,0,0)
        mott_json = json_message_converter.convert_ros_message_to_json(m)

        # test CommandRequestList
        crl = CommandRequestList()
        cr = CommandRequest()
        cr.id = 1
        cr.action = "mott"
        cr.args = mott_json
        crl.commands.append(cr)

        self.mr_command_req_callback(crl)

    def two_block_stack(self):
        table_z = -.230
        hp2_z = -0.548
        block_width = .0385

        crl = CommandRequestList()

        m = Mott()
        m.command = "mott"
        m.object_pose_topic = "ar_3"
        m.target_pose_topic = "target_1"

        m.object_pose.header.frame_id = "baxter"
        
        m.target_pose.header.frame_id = "baxter"
        m.target_pose.header.stamp = rospy.Time.now()
        m.target_pose.pose.position = Point(.6,-.5, table_z)
        # m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
        m.target_pose.pose.orientation = Quaternion(0,1,0,0)

        mott_json = json_message_converter.convert_ros_message_to_json(m)

        cr = CommandRequest()
        cr.id = "1"
        cr.action = "mott"
        cr.args = mott_json
        crl.commands.append(cr)

        m = Mott()
        m.command = "mott"
        m.object_pose_topic = "ar_5"
        m.target_pose_topic = "target_2"

        m.object_pose.header.frame_id = "baxter"
        
        m.target_pose.header.frame_id = "baxter"
        m.target_pose.header.stamp = rospy.Time.now()
        m.target_pose.pose.position = Point(.6,-.5, table_z+block_width)
        # m.target_pose.pose.position = Point(-0.1,0.725, hp2_z)
        m.target_pose.pose.orientation = Quaternion(0,1,0,0)
        
        mott_json = json_message_converter.convert_ros_message_to_json(m)

        cr = CommandRequest()
        cr.id = "2"
        cr.action = "mott"
        cr.args = mott_json
        crl.commands.append(cr)

        crl.parents = ["1"]
        crl.children = ["2"]

        # test CommandRequestList
        self.mr_command_req_callback(crl)

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
                "baxter"
                # "global_map"
            ),
            self.crq
        )
        self.robots.append(baxter_left)
        # self.baxter_left = baxter_left

        baxter_right = Robot("baxter_right",  1.0,
            Workspace(
                Point( 0.80,-0.80,-0.60),
                Point(-0.30, 0.10, 0.70),
                "baxter"
            ),
            self.crq
        )
        self.robots.append(baxter_right)

        height = 0.4
        gatlin = Robot("gatlin", 0.6,
            Workspace(
                Point(-5.0,-5.0,0.0),
                Point(5.0,5.0,height),
                "gatlin"
            ),
            self.crq
        )
        self.robots.append(gatlin)

        # youbot = Robot("youbot", 0.6, 
        #     Workspace(
        #         Point(-5.0,-5.0,0.0),
        #         Point(5.0,5.0,height),
        #         "base_link"
        #     ),
            # self.crq
        # )
        # self.robots.append(youbot)

        # height = 0.2
        # modbot = Robot("modbot", 0.6,
        #     Workspace(
        #         Point(-5.0,-5.0,0.0),
        #         Point(5.0,5.0,height),
        #         "base_link"
        #     ),
            # self.crq
        # )
        # self.robots.append(modbot)

    def display_workspaces(self):
        self.workspace_pub = rospy.Publisher("/workspace_markers", Marker, queue_size = 1)
        
        rate = rospy.Rate(30)
        # while not rospy.is_shutdown():
        for x in range(0,20):
            for i in range(0,len(self.robots)):
                self.display_workspace(self.robots[i], i)
            rate.sleep()

    def display_workspace(self, robot, i):
        workspace = robot.workspace
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
        marker.color.r = robot.color
        marker.color.b = 1-robot.color
        marker.color.a = 0.5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        self.workspace_pub.publish(marker)
        # rospy.logerr(marker)

    def mr_command_req_callback(self, cmd_req):
        rospy.logerr(cmd_req)
        self.crq_locked = True

        # recieve commands
        generated_cmd_ids = {}
        for cmd in cmd_req.commands:
            if cmd.action == "mott":
                cmd.args = cmd.args.replace("'", "\"")
                # rospy.logerr(cmd)
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

                # find path from object to target in connectivity graph
                path = self.wcg.find_shortest_path(mott.object_pose_topic, mott.target_pose_topic)
                rospy.logerr(path)

                

                # generate actions based on optimal path
                if path != None:
                    gci = self.wcg.generate_commands(path)
                    # rospy.logerr(gci)
                    generated_cmd_ids[cmd.id] = gci

                

            elif cmd.action == "move_base":
                pass

        # use dependencies to build update crq
        # rospy.logerr(generated_cmd_ids)
        for i in range(0, len(cmd_req.parents)):
            parent = cmd_req.parents[i]
            child = cmd_req.children[i]
            # rospy.logerr(parent)
            # rospy.logerr(child)

            for parent_id in generated_cmd_ids[parent]:
                for child_id in generated_cmd_ids[child]:
                    # rospy.logerr("%s <- %s" % (parent_id, child_id))
                    self.crq.add_dependency(parent_id, child_id)

        self.crq_locked = False
        
        self.wcg.draw()
        self.wcg.show()

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
            # rospy.logerr("%r, %r, %r," % (inx, iny, inz))
            return inx and iny and inz

        def getCenterDimensions(self):
            p1 = vector3_to_numpy(self.p1)
            p2 = vector3_to_numpy(self.p2)
            center = (p1+p2)/2
            dimensions = abs(p1-center)*2
            return center, dimensions


class Robot:
    def __init__(self, name, color, workspace, cr_queue):
        self.name = name
        self.color = color
        self.workspace = workspace
        self.cmd_req_queue = cr_queue
        # publishers for all service requests
        self.mott_pub = rospy.Publisher("/%s_mott" % name, Mott, queue_size = 1)
        self.mott_command_pub = rospy.Publisher("/%s_mott_command" % name, String, queue_size = 1)

        # subscribers for all service responses
        self.response_sub = rospy.Subscriber("/%s_mott_response" % name, String, self.mott_response_callback, queue_size = 1)

        self.current_cmd = None

        # while not rospy.is_shutdown() :
        #     if self.current_cmd == None :
        #         new_cmd = self.cmd_req_queue.request_command(self.name)
        #         self.executue_command(new_cmd)
        #     rospy.sleep(.05)

    def mott_response_callback(self, resp):

        rospy.logerr("mott_response_cb : "+resp.data)

        if "finish" in resp.data :
            rospy.logerr("CMD FINISHED on %s" % self.name)
            # rospy.logerr(self.name)
            # rospy.logerr(self.current_cmd)
            self.cmd_req_queue.robot_finished(self.name, self.current_cmd)
            self.current_cmd = None
        elif "quit" in resp.data :
            self.cmd_req_queue.robot_quit(self.name, self.current_cmd)
            self.current_cmd = None
   
    def execute_command(self, cmd):
        if cmd != None:
            if cmd.action == "mott":
                self.current_cmd = deepcopy(cmd)
                # rospy.logerr("self.current_cmd")
                # rospy.logerr(self.current_cmd)
                m = json_message_converter.convert_json_to_ros_message('gatlin/Mott', cmd.args)
                self.publish_mott(m)

    def publish_mott(self, m):
        self.mott_pub.publish(m)
        rospy.logerr("PUBLISHED MOTT to %s" % self.name)
        # rospy.logerr(m)            

# class HP:
#     def __init__(self, name, color, workspace):
#         self.transform = TransformStamped()

class WorkspaceConnectivityGraph:
    def __init__(self, robots, crq, tfl):
        # rospy.init_node('WorkspaceConnectivityGraph')
        self.tfl = tfl

        self.robots = robots
        self.crq = crq

        self.color_map = {}
        for r in self.robots:
            self.color_map[r.name] = r.color

        self.G = nx.DiGraph()

        self.tfl = tfl

        self.cmd_idx = 0

        self.fixed_frame = "global_map"
        # self.fixed_frame = "base"
        self.dm = DynamicManager(self.tfl)
        self.dm.add_ol_sub("/server/ar_marker_list")

        self.objects = []
        self.targets = []
        self.hps = []

        self.init_handoff_zones()

        # self.update_distances()
        # self.draw()
        # self.show()

    def generate_commands(self, path):
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

        crl = CommandRequestList()
        generated_cmd_ids = []
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
            # rospy.logerr(m)

            m_json = json_message_converter.convert_ros_message_to_json(m)

            cr = CommandRequest()
            cr.id = self.cmd_idx
            self.cmd_idx += 1
            cr.action = "mott"
            cr.args = m_json
            crl.commands.append(cr)

            self.crq.add_command_req(cr, robot_name)
            generated_cmd_ids.append(cr.id)
            if i > 0:
                self.crq.add_dependency(cr.id-1, cr.id)

        return generated_cmd_ids


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

    def find_robot(self, robot_name):
        for r in self.robots:
            if r.name == robot_name:
                return r
        return None

    def init_handoff_zones(self):
        # xyz point and width, length
        table_z = -.240
        hp_1 = Workspace(
            Point(0.70,-0.10, table_z-.01),
            Point(0.50, 0.10, table_z+.01),
            "baxter"
        )
        self.add_hp("hp_1", hp_1)

        hp2_z = -.548
        hp_2 = Workspace(
            Point(-0.45, 0.60, hp2_z-.01),
            Point(0.25, 0.85, hp2_z+.01),
            "baxter"
        )
        self.add_hp("hp_2", hp_2)

    def add_hp(self, hp_name, hp_workspace):
        center, dimensions = hp_workspace.getCenterDimensions()
        init_hp_pose = PoseStamped()
        init_hp_pose.header.frame_id = hp_workspace.reference_frame
        init_hp_pose.header.stamp = rospy.Time.now()
        init_hp_pose.pose.position.x = center[0]
        init_hp_pose.pose.position.y = center[1]
        init_hp_pose.pose.position.z = center[2]

        init_hp_pose.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

        # create dp
        hp_dp = self.dm.create_dp(self.fixed_frame)
        hp_dp.subscribe_name(hp_name)
        hp_dp.set_pose(init_hp_pose)
        # rospy.logerr(hp_dp.ps)

        self.hps.append(hp_dp)

        # given a hp determine if it is in the robots wksp
        for robot in self.robots:
            local_ps = self.transform_pose(robot.workspace.reference_frame, hp_dp.ps)
            inside = robot.workspace.inside_workspace(local_ps.pose.position)
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

        if isEqualPoint(obj_pose.pose.position, Point(0,0,0)):
            rospy.logerr("obj_pose is identity")
            while not rospy.is_shutdown() and isEqualPoint(object_dp.ps.pose.position, Point(0,0,0)):
                rospy.logerr("object_dp not found")
                rospy.sleep(.1)
        else:
            object_dp.set_pose(obj_pose)
            # rospy.logerr(obj_pose)
        rospy.sleep(.2)

        self.objects.append(object_dp)
        # rospy.logerr(object_dp.ps)

        # given a obj determine if it is in the robots wksp
        for robot in self.robots:
            local_ps = self.transform_pose(robot.workspace.reference_frame, object_dp.ps)
            inside = robot.workspace.inside_workspace(local_ps.pose.position)
            # if inside then add an edge
            if inside:
                robot_to_obj = self.getDist(robot, object_dp)
                # rospy.logerr(robot_to_obj)
                self.add_obj_edge(robot.name, obj_topic, robot_to_obj)

    def add_obj_edge(self, robot, obj_topic, robot_to_obj):
        self.color_map[obj_topic] = 0.5
        self.G.add_edge(obj_topic, robot, distance=robot_to_obj)

    def add_target(self, target_topic, target_pose):
        # create_dp
        target_dp = self.dm.create_dp(self.fixed_frame)
        target_dp.subscribe_name(target_topic)

        if isEqualPoint(target_pose.pose.position, Point(0,0,0)):
            rospy.logerr("target_pose is identity")
            while not rospy.is_shutdown() and isEqualPoint(target_dp.ps.pose.position, Point(0,0,0)):
                rospy.logerr("target_dp not found")
                rospy.sleep(.1)
        else:
            target_dp.set_pose(target_pose)
        rospy.sleep(.2)

        self.targets.append(target_dp)

        # given a target determine if it is in the robots wksp
        for robot in self.robots:
            local_ps = self.transform_pose(robot.workspace.reference_frame, target_dp.ps)
            inside = robot.workspace.inside_workspace(local_ps.pose.position)
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
