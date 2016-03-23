#!/usr/bin/env python
# Team: zic; Names: Zach Vinegar, Isaac Qureshi, Chris Silvia
import rospy
import numpy as np
from std_msgs.msg import *
from geometry_msgs.msg import *
from gatlin.srv import *
from gatlin.msg import *
from baxter_core_msgs.msg import *
from baxter_core_msgs.srv import *
from baxter_interface import *
from config import *
from copy import deepcopy
from tf.transformations import *

class RobotInterface():
    def __init__(self):
        rospy.init_node('position_control')
        rospy.loginfo("Initialized Position Control")

        rospy.loginfo("Beginning to enable robot")
        self.baxter = RobotEnable()
        rospy.loginfo("Enabled Robot")
        
        self.limb_name = "left"

        self.hand_pose = Pose()

        self.limb = Limb(self.limb_name)
        
        self.gripper = Gripper(self.limb_name)
    

        rospy.Subscriber("/robot/limb/"+self.limb_name+"/endpoint_state", EndpointState, self.respondToEndpoint)
        
        move_robot_service = createService('baxter/'+self.limb_name+'/move_robot', MoveRobot, self.handle_move_robot, "")

        # self.position_srv = createService('end_effector_position', EndEffectorPosition, self.get_position_response, self.limb)

        try :
            rospy.loginfo("Initializing service proxy for /SolvePositionIK...")
            ns = "ExternalTools/"+self.limb_name+"/PositionKinematicsNode/IKService"
            rospy.wait_for_service(ns, 5.0)
            self.iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
            rospy.loginfo("Initialized service proxy for /SolvePositionIK...")
        except rospy.ServiceException, e:
            rospy.logerr("Service Initializing Failed: {0}".format(e))

        print "Ready to move baxter " +self.limb_name

        rospy.spin()

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
            success = self.MoveToPoseWithIntermediate(self.limb_name, req.pose)

        elif req.action == MOVE_TO_POSE :
            rospy.loginfo("Trying to Move To Pose")
            success = self.MoveToPose(self.limb_name, req.pose, "FAILED MoveToPose")

        elif req.action == MOVE_TO_POS :
            rospy.loginfo("Trying to Move To Pos")

            new_pose = Pose()
            try:
                new_pose = deepcopy(self.initial)
            except AttributeError:
                new_pose = deepcopy(self.hand_pose)
                self.initial = deepcopy(self.hand_pose)

            new_pose.position = deepcopy(req.pose.position)
            # success = self.MoveToPose(req.limb, new_pose, "FAILED MoveToPose")
            success = self.MoveToPoseWithIntermediate(self.limb_name, new_pose)
            rospy.loginfo("Moved to pos: %r" % success)

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

        iksvc = self.iksvc

        try :
            ns = "ExternalTools/"+limb+"/PositionKinematicsNode/IKService"
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
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
        RobotInterface()
    except rospy.ROSInterruptException:
        pass
