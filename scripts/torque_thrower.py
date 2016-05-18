#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
"""
Baxter RSDK Joint Torque Example: joint springs
"""

import argparse

import rospy, math
import numpy as np

from dynamic_reconfigure.server import (
    Server,
)
from std_msgs.msg import (
    Empty,
)

import baxter_interface

from baxter_examples.cfg import (
    JointSpringsExampleConfig,
)
from baxter_interface import CHECK_VERSION
from config import *
from gatlin.srv import *


class TorqueThrower(object):
    """
    Virtual Joint Springs class for torque example.

    @param limb: limb on which to run joint springs example
    @param reconfig_server: dynamic reconfigure server

    TorqueThrower class contains methods for the joint torque example allowing
    moving the limb to a neutral location, entering torque mode, and attaching
    virtual springs.
    """
    def __init__(self, limb, reconfig_server):
        self._dyn = reconfig_server

        # control parameters
        self._rate = 1000.0  # Hz
        self._missed_cmds = 20.0  # Missed cycles before triggering timeout

        # create our limb instance
        self._limb = baxter_interface.Limb(limb)
        self.limb = limb
        self.gripper = baxter_interface.Gripper(limb, CHECK_VERSION)

        # initialize parameters
        self._springs = dict()
        self._damping = dict()
        self._start_angles = dict()

        self.enabled = False
        throw_service = createService("/baxter_%s/throw" % limb, Action, self.throw_srv)

        # create cuff disable publisher
        cuff_ns = 'robot/limb/' + limb + '/suppress_cuff_interaction'
        self._pub_cuff_disable = rospy.Publisher(cuff_ns, Empty, queue_size=1)

        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        print("Running. Ctrl-c to quit")

    def throw_srv(self, req):
        rospy.logerr("Got Throw Service Req")
        self.thrown = False
        self.enabled = False
        self.move_to_release()
        self.prepare_for_throw()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.enabled:
            rate.sleep
        
        rospy.logerr(ActionResponse(True))

        return ActionResponse(True)

    def move_to_release(self):

        _start_angles = {'left_w0': -0.2, 'left_w1': .4, 'left_w2': 0.0, 'left_e0': 0.11, 'left_e1': 1.69, 'left_s0': -0.9, 'left_s1': -0.97}
        # _start_angles = {'left_w0': 3.05, 'left_w1': -0.19, 'left_w2': -0.08, 'left_e0': 0.17, 'left_e1': 1.56, 'left_s0': -0.94, 'left_s1': -0.77}

        if self.limb == 'right':
            _start_angles = mirror_left_arm_joints(_start_angles)

        # self.gripper.command_position(100, block=True)
        # rospy.logerr(_start_angles)
        self._limb.move_to_joint_positions(_start_angles)
        # rospy.sleep(1)
        # self.gripper.command_position(0, block=True)

        rospy.logerr("Moved to start_angles")


    def prepare_for_throw(self):
        safty_buffer = math.radians(30.0)
        w1_min = -1.571
        w1_max = 2.094

        linear_speed = .1
        # rospy.logerr("linear_speed (m/sec): %.3f", linear_speed)

        self.link_len = 0.31 # need to measure
        angular_speed = linear_speed/self.link_len
        if angular_speed > 4.0:
            angular_speed = 4.0
        # print "angular_speed (deg/sec)"
        # print  math.degrees(angular_speed)
        rospy.logerr("angular_speed (rad/sec): %.3f", angular_speed)

        back_swing = math.radians(70.0)
        follow_through = math.radians(60.0)
        open_gripper_time = .10 # need to calibrate

        current_joints = self._limb.joint_angles()

        # set self.release_angle as current joint angle
        self.release_angle = self._limb.joint_angle(self.limb+"_w1")
        self.release_angle = self.release_angle

        # set self.open_angle to compensate for open gripper time
        open_offset = open_gripper_time * angular_speed
        self.open_angle = self.release_angle + open_offset

        # set self.stop_angle allowing for follow through 
        self.stop_angle = self.release_angle - follow_through
        # make sure follow through won't hit joint limit
        if self.stop_angle < w1_min + safty_buffer :
            self.stop_angle = w1_min + safty_buffer

        # move the wrist bend to backswing pos
        # move the wrist twist to neutral pos
        current_joints[self.limb+'_w1'] += back_swing
        if current_joints[self.limb+'_w1'] > w1_max :
            current_joints[self.limb+'_w1'] = w1_max

        current_joints[self.limb+'_w2'] = 0.0
        self._limb.move_to_joint_positions(current_joints)
        # rospy.sleep(.5)

        self.opened = False
        self.released = False

        self.target_velocity = -2.0
        # self.torque = self.target_velocity/15.0
        self.torque = -1.8

        rospy.logerr("Prepared for throw.")

        self.enabled = True
        self._start_angles = self._limb.joint_angles()


    def _update_parameters(self):
        for joint in self._limb.joint_names():
            self._springs[joint] = self._dyn.config[joint[-2:] +
                                                    '_spring_stiffness']
            self._damping[joint] = self._dyn.config[joint[-2:] +
                                                    '_damping_coefficient']


    def _set_max_parameters(self):
        self._springs = {self.limb+'_w0': 9.0, self.limb+'_w1': 4.0, self.limb+'_w2': 4.0, self.limb+'_e0': 15.0, self.limb+'_e1': 15.0, self.limb+'_s0': 30.0, self.limb+'_s1': 30.0}
        self._damping = {self.limb+'_w0': 1.5, self.limb+'_w1': 1.5, self.limb+'_w2': 1.0, self.limb+'_e0': 7.5, self.limb+'_e1': 5.0, self.limb+'_s0': 10.0, self.limb+'_s1': 7.5}

    def _update_forces(self):
        """
        Calculates the current angular difference between the start position
        and the current joint positions applying the joint torque spring forces
        as defined on the dynamic reconfigure server.
        """
        # get latest spring constants
        # self._update_parameters()

        # disable cuff interaction
        self._pub_cuff_disable.publish()

        # create our command dict
        cmd = dict()
        # record current angles/velocities
        cur_pos = self._limb.joint_angles()
        cur_vel = self._limb.joint_velocities()
        # calculate current forces
        for joint in self._start_angles.keys():
            # spring portion
            cmd[joint] = self._springs[joint] * (self._start_angles[joint] -
                                                   cur_pos[joint])
            # damping portion
            cmd[joint] -= self._damping[joint] * cur_vel[joint]

            if joint == self.limb+"_w1":

                current_angle = self._limb.joint_angle(self.limb+"_w1")
                current_velocity = self._limb.joint_velocity(self.limb+"_w1")

                current_effort = self._limb.joint_effort(self.limb+"_w1")
                current_cmd = cmd[joint]

                # opens gripper slightly before self.release_angle to account for open gripper time
                if not self.opened and self.open_angle > current_angle : 
                    self.gripper.command_position(100, block=False)
                    self.opened = True

                # should release at correct pos/vel
                if not self.released and self.release_angle > current_angle :
                    actual_release_pos = self._limb.endpoint_pose()['position']
                    # actual_release_vel = np.array(self._limb.endpoint_velocity()['linear'])
                    actual_release_speed = current_velocity #* self.link_len
                    actual_release_speed_error = self.target_velocity - current_velocity #* self.link_len
                    self.released = True
                    # rospy.loginfo(("actual_release_pos:", actual_release_pos))
                    rospy.loginfo("actual_release_speed: %.3f", actual_release_speed)
                    rospy.loginfo("actual_release_speed_error: %.3f", actual_release_speed_error)

                # stops at the self.stop_angle
                if self.stop_angle > current_angle :
                    # self._limb.set_joint_velocities(zero_dict)
                    self.enabled = False
                    self._limb.exit_control_mode()
                    return

                vel_error = self.target_velocity - current_velocity
                # rospy.loginfo("vel_error: %.3f", vel_error)
                # rospy.loginfo("scaled_error: %.3f", 1+vel_error/self.target_velocity*.011)
                # self.torque += vel_error*.0055
                # self.torque *= 1+vel_error/self.target_velocity*.05
                cmd[joint] = self.torque

        # command new joint torques
        self._limb.set_joint_torques(cmd)

    def move_to_neutral(self):
        """
        Moves the limb to neutral location.
        """
        # self._limb.move_to_neutral()

    def attach_springs(self):
        """
        Switches to joint torque mode and attached joint springs to current
        joint positions.
        """
        # record initial joint angles
        self._start_angles = self._limb.joint_angles()

        # set control rate
        control_rate = rospy.Rate(self._rate)

        # for safety purposes, set the control rate command timeout.
        # if the specified number of command cycles are missed, the robot
        # will timeout and disable
        self._limb.set_command_timeout((1.0 / self._rate) * self._missed_cmds)

        self._set_max_parameters()

        # loop at specified rate commanding new joint torques
        while not rospy.is_shutdown():
            # self._update_parameters()
            if not self._rs.state().enabled:
                rospy.logerr("Joint torque example failed to meet "
                             "specified control rate timeout.")
                break

            if not self.enabled:
                # rospy.logerr("disabled")
                continue

            self._update_forces()
            control_rate.sleep()

    def clean_shutdown(self):
        """
        Switches out of joint torque mode to exit cleanly
        """
        print("\nExiting example...")
        self._limb.exit_control_mode()
        if not self._init_state and self._rs.state().enabled:
            print("Disabling robot...")
            self._rs.disable()


def main():
    """RSDK Joint Torque Example: Joint Springs

    Moves the specified limb to a neutral location and enters
    torque control mode, attaching virtual springs (Hooke's Law)
    to each joint maintaining the start position.

    Run this example on the specified limb and interact by
    grabbing, pushing, and rotating each joint to feel the torques
    applied that represent the virtual springs attached.
    You can adjust the spring constant and damping coefficient
    for each joint using dynamic_reconfigure.
    """
    # arg_fmt = argparse.RawDescriptionHelpFormatter
    # parser = argparse.ArgumentParser(formatter_class=arg_fmt,
    #                                  description=main.__doc__)
    # parser.add_argument(
    #     '-l', '--limb', dest='limb', required=True, choices=['left', 'right'],
    #     help='limb on which to attach joint springs'
    # )
    # args = parser.parse_args(rospy.myargv()[1:])

    # print("Initializing node... ")
    rospy.init_node("baxter_thrower", anonymous=True)
    limb_name = rospy.get_param('~limb')
    # limb_name = "right"

    dynamic_cfg_srv = Server(JointSpringsExampleConfig,
                             lambda config, level: config)
    js = TorqueThrower(limb_name, dynamic_cfg_srv)
    # register shutdown callback
    rospy.on_shutdown(js.clean_shutdown)
    js.move_to_neutral()
    js.attach_springs()

if __name__ == "__main__":
    main()
