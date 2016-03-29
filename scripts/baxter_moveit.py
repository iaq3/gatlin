#!/usr/bin/env python
import rospy, sys
import moveit_commander
from std_msgs.msg import *
from geometry_msgs.msg import *
from moveit_commander import *
from moveit_msgs.msg import *
from trajectory_msgs.msg import *
from tf.transformations import *
from gatlin.msg import *
from gatlin.srv import *
from config import *
from copy import deepcopy

class BaxterMoveIt:
    def __init__(self):
        # Give the launch a chance to catch up
        # rospy.sleep(5)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('Baxter_Controller')
        rospy.loginfo("Launched Baxter Controller")

        self.REFERENCE_FRAME = 'base'
        self.done = True

        self.test_pose_publisher = rospy.Publisher('/test_arm_pose', PoseStamped)

        # rospy.Subscriber("left/arm_target_pose", PoseStamped, self.move_arm_to_pose, queue_size=1)
        self.robot_name = "baxter"
        left_move_arm_service = createService("%s/left/move/arm" % (self.robot_name), MoveRobot, self.handle_move_arm_left)
        right_move_arm_service = createService("%s/right/move/arm" % (self.robot_name), MoveRobot, self.handle_move_arm_right)

        # We need a tf listener to convert poses into arm reference base
        self.tfl = tf.TransformListener()

        # Initialize the move group for the left and right arm
        self.left_arm = MoveGroupCommander('left_arm')
        self.right_arm = MoveGroupCommander('right_arm')

        # Initialize the move group for the left and right gripper
        self.left_gripper = MoveGroupCommander("left_hand")
        self.right_gripper = MoveGroupCommander("right_hand")

        # Get the name of the end-effector link
        left_end_effector_link = self.left_arm.get_end_effector_link()
        right_end_effector_link = self.right_arm.get_end_effector_link()

        self.robot = moveit_commander.RobotCommander()

        # Allow replanning to increase the odds of a solution
        self.left_arm.allow_replanning(True)
        self.right_arm.allow_replanning(True)

        # Set the planner
        self.left_arm.set_planner_id("RRTConnectkConfigDefault")
        self.right_arm.set_planner_id("RRTConnectkConfigDefault")

        # Set the right and left reference frame
        self.left_arm.set_pose_reference_frame(self.REFERENCE_FRAME)
        self.right_arm.set_pose_reference_frame(self.REFERENCE_FRAME)

        # Give the scene a chance to catch up
        rospy.sleep(1)

        # Allow some leeway in position (meters) and orientation (radians)
        # USELESS; do not work on pick and place! Explained on this issue:
        # https://github.com/ros-planning/moveit_ros/issues/577
        self.left_arm.set_goal_position_tolerance(0.005)
        self.right_arm.set_goal_orientation_tolerance(0.05)
        
        self.left_arm.set_goal_position_tolerance(0.005)
        self.right_arm.set_goal_orientation_tolerance(0.05)

        # Allow 2 seconds per planning attempt
        self.left_arm.set_planning_time(2.0)
        self.right_arm.set_planning_time(2.0)

        rospy.spin()

    def MoveToPoseWithIntermediate(self, limb, ps, offsets) :
        success = False
        for offset in offsets:
            # interpose = getOffsetPose(hand_pose, offset)
            interpose = getOffsetPose(ps, offset)
            success = self.MoveToPose(limb, interpose, "MoveToIntermediatePose")
            rospy.sleep(1)

        success = self.MoveToPose(limb, ps, "MoveToPose")

        return success

    def MoveToPose(self, limb, ps, name) :
        newpose = self.transform_pose(self.REFERENCE_FRAME, ps)
        down = Quaternion(-0.00035087, 0.73273, 0.00030411, 0.68052)
        newpose.pose.orientation = down
        # newpose.pose.position.z -= .015

        if self.move_arm_to_pose(newpose, limb) :
            rospy.loginfo("SUCCEEDED: %s" % name)
            return True
        else :
            rospy.logerr("FAILED %s" % name)
            return False

    def move_arm_to_pose(self, limb, ps):
        arm_target_pose = deepcopy(ps)
        arm_target_pose.header.stamp = rospy.Time.now()
        
        self.test_pose_publisher.publish(arm_target_pose)
        
        arm = self.left_arm if limb == "left" else self.right_arm
        arm.set_pose_target(arm_target_pose)
        success = arm.go()

        return success

    def handle_move_arm_left(self, req):
        self.handle_move_arm(req, "left")

    def handle_move_arm_right(self, req):
        self.handle_move_arm(req, "right")

    def handle_move_arm(self, req, limb):
        success = True
        gripper = self.left_gripper if limb == "left" else self.right_gripper

        if req.action == OPEN_GRIPPER:
            rospy.loginfo("Beginning to open gripper")
            gripper.open(block=True)
            rospy.loginfo("Opened Gripper")

        elif req.action == CLOSE_GRIPPER :
            rospy.loginfo("Beginning to close Gripper")
            gripper.close(block=True)
            rospy.loginfo("Closed Gripper")

        elif req.action == MOVE_TO_POSE_INTERMEDIATE :
            rospy.loginfo("Trying to Move To Pose With Intermediate")
            offsets = [Vector3(0,0,.07)]
            success = self.MoveToPoseWithIntermediate(limb, req.ps, offsets)

        elif req.action == MOVE_TO_POSE :
            rospy.loginfo("Trying to Move To Pose")
            success = self.MoveToPose(limb, req.ps, "FAILED MoveToPose")

        elif req.action == RESET_ARM :
            rospy.loginfo("Trying to Move To Rest Pose")
            success = self.move_arm_to_pose(limb, self.rest_pose)
            # success = self.MoveToPose(self.rest_pose, "FAILED MoveToRestPose")

        # elif req.action == MOVE_TO_POS :
        #   rospy.loginfo("Trying to Move To Pos")

        #   new_pose = Pose()
        #   if req.limb == 'left':
        #       try:
        #           self.initial_left
        #           new_pose = deepcopy(self.initial_left)
        #       except AttributeError:
        #           new_pose = deepcopy(self.hand_pose_left)
        #           self.initial_left = deepcopy(self.hand_pose_left)
        #   elif req.limb == 'right':
        #       try:
        #           self.initial_right
        #           new_pose = deepcopy(self.initial_right)
        #       except AttributeError:
        #           new_pose = deepcopy(self.hand_pose_right)
        #           self.initial_right = deepcopy(self.hand_pose_right)

        #   new_pose.position = deepcopy(req.pose.position)
        #   # success = self.MoveToPose(req.limb, new_pose, "FAILED MoveToPose")
        #   success = self.MoveToPoseWithIntermediate(req.limb, new_pose)
        #   rospy.loginfo("Moved to pos: %r" % success)

        else :
            success = False
            rospy.logerr("invalid action")

        return MoveRobotResponse(success)


    def orientation_cb(self, data):
        if(data.x == -1.0 and data.y == -2.0):
            print "################################################"
            print "###################  Orientation!  #############"
            print "################################################"
            deg = data.z*15
            rad = -deg * pi/180 + pi/2
            print rad
            q = quaternion_from_euler(0,rad,0)
            self.current_pose.pose.orientation.x = q[0]
            self.current_pose.pose.orientation.y = q[1]
            self.current_pose.pose.orientation.z = q[2]
            self.current_pose.pose.orientation.w = q[3]
        else:
            return

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

if __name__ == "__main__":
    BaxterMoveIt()