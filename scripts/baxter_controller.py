#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from gatlin.srv import *
from gatlin.msg import *
from baxter_core_msgs.msg import *
from baxter_core_msgs.srv import *
from baxter_interface import *
from config import *
from copy import deepcopy
from tf.transformations import *
import cv2, cv_bridge, rospkg

class BaxterController():
    def __init__(self):
        rospy.init_node('baxter_controller')
        rospy.loginfo("Initialized Baxter Control")

        rospy.loginfo("Beginning to enable robot")
        self.baxter = RobotEnable()
        rospy.loginfo("Enabled Robot")
        
        self.hand_pose = Pose()

        self.limb_name = rospy.get_param('~limb')
        # self.limb_name = "left"
        self.robot_name = "baxter"

        self.limb = Limb(self.limb_name)
        
        self.gripper = Gripper(self.limb_name)

        rospy.Subscriber("/robot/limb/"+self.limb_name+"/endpoint_state", EndpointState, self.respondToEndpoint)
        
        # self.arm_target_sub = rospy.Subscriber("/baxter_arm_target_pose_%s" % self.limb_name, PoseStamped, self.arm_target_callback, queue_size=1)
        
        self.move_robot_service = createService("%s_%s/move_robot" % (self.robot_name, self.limb_name), MoveRobot, self.handle_move_robot)

        # self.position_srv = createService("%s/end_effector_position" % self.limb, EndEffectorPosition, self.get_position_response)

        try :
            ns = "ExternalTools/"+self.limb_name+"/PositionKinematicsNode/IKService"
            rospy.wait_for_service(ns, 5.0)
            self.iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
            rospy.loginfo("Initialized service proxy for /SolvePositionIK...")
        except rospy.ServiceException, e:
            rospy.logerr("Service Initializing Failed: {0}".format(e))

        print "Ready to move baxter " +self.limb_name

        self.show_ARmarker_face()

        self.test_pose_pub = rospy.Publisher('/test_arm_target', PoseStamped, queue_size = 1)

        rospy.spin()

    def arm_target_callback(self, ps):
        rospy.logerr(ps)
        req = MoveRobotRequest()
        # req.action = MOVE_TO_POSE_INTERMEDIATE
        # req = {}
        req.action = MOVE_TO_POS
        req.ps = ps
        self.handle_move_robot(req)

    def show_ARmarker_face(self):
        rospack = rospkg.RosPack()
        img_name = "marker3_baxter.png"
        img = cv2.imread(rospack.get_path('gatlin')+"/img/"+img_name)
        img_msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")

        self.display_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        self.display_pub.publish(img_msg)
        rospy.loginfo(img_name+" sent to Baxter Display")
        rospy.sleep(1)

    def get_position_response(self, args):
        position = self.get_position()
        return EndEffectorPositionResponse(Vector3(position[0],position[1],position[2]))

    # def get_velocity_response(self, args):
    #     velocity = self.get_velocity()
    #     return EndEffectorVelocityResponse(Vector3(velocity[0], velocity[1], velocity[2]))

    def get_position(self):
        return np.array(self.limb.endpoint_pose()['position']) 

    # def get_velocity(self):
    #     return np.array(self.limb.endpoint_velocity()['linear'])

    def respondToEndpoint(self, EndpointState) :
        self.hand_pose = deepcopy(EndpointState.pose)


    def handle_move_robot(self, req):
        rospy.sleep(1)
        success = True
        
        gripper = self.gripper

        if req.action == MOVE_TO_POSE_INTERMEDIATE or req.action == MOVE_TO_POSE or req.action == MOVE_TO_POS:
            rospy.logerr(req.ps)
            self.test_pose_pub.publish(req.ps)

        if req.action == OPEN_GRIPPER:
            rospy.loginfo("Beginning to open gripper")
            # rospy.sleep(GRIPPER_WAIT)
            gripper.open(block=True)
            # rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Opened Gripper")

        elif req.action == CLOSE_GRIPPER :
            rospy.loginfo("Beginning to close Gripper")
            # rospy.sleep(GRIPPER_WAIT)
            gripper.close(block=True)
            # rospy.sleep(GRIPPER_WAIT)
            rospy.loginfo("Closed Gripper")

        elif req.action == MOVE_TO_POSE_INTERMEDIATE :
            rospy.loginfo("Trying to Move To Pose")
            success = self.MoveToPoseWithIntermediate(self.limb_name, req.ps.pose)

        elif req.action == MOVE_TO_POSE :
            rospy.loginfo("Trying to Move To Pose")
            success = self.MoveToPose(self.limb_name, req.ps.pose, "FAILED MoveToPose")

        elif req.action == MOVE_TO_POS :
            rospy.loginfo("Trying to Move To Pos")

            new_pose = Pose()
            try:
                new_pose = deepcopy(self.initial)
            except AttributeError:
                new_pose = deepcopy(self.hand_pose)
                self.initial = deepcopy(self.hand_pose)

            new_pose.position = deepcopy(req.ps.pose.position)
            # success = self.MoveToPose(req.limb, new_pose, "FAILED MoveToPose")
            success = self.MoveToPoseWithIntermediate(self.limb_name, new_pose)
            rospy.loginfo("Moved to pos: %r" % success)

        elif req.action == RESET_ARM :
            rospy.loginfo("Trying to Reset Arm")
            rest_pose = Pose()
            rest_pose.position = Point(0.5, 0.5, 0.1)
            rest_pose.orientation = Quaternion(0.0,1.0,0.0,0.0)

            if self.limb_name == "right": rest_pose.position.y *= -1

            success = self.MoveToPose(self.limb_name, rest_pose, "FAILED MoveToPose")

        else :
            print "invalid action"

        return MoveRobotResponse(success)

    def getCurrentPose(self, arm):
        pos = arm.endpoint_pose()['position']
        ori = arm.endpoint_pose()['orientation']
        p = Pose()
        p.position = Point(pos[0],pos[1],pos[2])
        p.orientation = Quaternion(ori[0],ori[1],ori[2],ori[3])
        return p

    def MoveToPoseWithIntermediate(self, limb, pose, inter1=True, inter2=True, inter3=False) :
        arm = self.limb
        hand_pose = self.getCurrentPose(arm)
        if inter1 :
            interpose1 = self.getOffsetPose(hand_pose, .07)
            b1 = self.MoveToPose(limb, interpose1, "MoveToIntermediatePose")
        if inter2 :
            interpose2 = self.getOffsetPose(pose, .07)
            b2 = self.MoveToPose(limb, interpose2, "MoveToIntermediatePose")
        if inter3 :
            interpose2 = self.getOffsetPose(pose, .01)
            b3 = self.MoveToPose(limb, interpose2, "MoveToRightAbovePose")
        
        b4 = self.MoveToPose(limb, pose, "MoveToPose")
        if not b4:
            self.MoveToPose(limb, hand_pose, "MoveToPose")
        return b4

    def MoveToPose(self, limb, pose, name) :
        joint_solution = self.inverse_kinematics(limb, pose)
        if joint_solution != [] :
            self.moveArm(limb, joint_solution)
            rospy.loginfo("SUCCEEDED: %s" % name)
            # rospy.sleep(MOVE_WAIT)
            return True
        else :
            rospy.logerr("FAILED %s" % name)
            return False

    def getOffsetPose(self, pose, offset) :
        offsetpose = deepcopy(pose)
        q = pose.orientation
        off = np.dot(
            quaternion_matrix([q.x,q.y,q.z,q.w]),
            np.array([0,0,-offset,1]).T)
        off = off[:3]/off[3]
        offsetpose.position.x += off[0]
        offsetpose.position.y += off[1]
        offsetpose.position.z += off[2]
        return offsetpose

    def moveArm (self, limb, joint_solution) :
        arm = Limb(limb)
        arm.move_to_joint_positions(joint_solution, timeout=5.0)
        rospy.sleep(0.01)

    def inverse_kinematics(self, limb, ourpose) :
        ikreq = SolvePositionIKRequest()

        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            limb : PoseStamped(
                header = hdr,
                pose = ourpose
            ),
        }
        ikreq.pose_stamp.append(poses[limb])

        try :
            ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
            rospy.wait_for_service(ns, 5.0)
            resp = self.iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return []
        if (resp.isValid[0]):
            print("SUCCESS - Valid Joint Solution Found:")
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            return limb_joints
        else :
            rospy.logerr("Invalid pose")
            return []

if __name__ == '__main__':
    try:
        BaxterController()
    except rospy.ROSInterruptException:
        pass
