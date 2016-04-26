#!/usr/bin/env python

import rospy
from sensor_msgs.msg import *

class Publisher:
    def __init__(self):
        rospy.init_node('publisher')

        self.pub = rospy.Publisher("/test/camera_info", CameraInfo, queue_size=1)

        i = CameraInfo()
        # i.header.stamp = rospy.Time.now()
        i.header.frame_id = "/left_hand_camera"
        i.distortion_model = "plumb_bob"
        i.D = [0.007374918478992584, -0.0622448609493766, 0.003190829993434288, -0.00015208871579441667, 0.02100990382030653]
        # i.K = [400.57915652712325, 0.0, 611.0776917145643, 0.0, 401.429294953449, 399.0709219799418, 0.0, 0.0, 1.0]
        i.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # i.P = [400.57915652712325, 0.0, 291.07769171456425, 0.0, 0.0, 401.429294953449, 199.07092197994177, 0.0, 0.0, 0.0, 1.0, 0.0]
        # i.binning_x = 1
        # i.binning_y = 1
        # i.roi.x_offset = 320
        # i.roi.y_offset = 200
        # i.roi.height = 400
        # i.roi.width = 640
        # i.roi.do_rectify = False

        # from kinect camera info
        # i.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        # i.K = [262.5, 0.0, 159.5, 0.0, 262.5, 119.5, 0.0, 0.0, 1.0]
        # i.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # i.P = [262.5, 0.0, 159.5, 0.0, 0.0, 262.5, 119.5, 0.0, 0.0, 0.0, 1.0, 0.0]
        i.binning_x = 0
        i.binning_y = 0
        i.roi.x_offset = 0
        i.roi.y_offset = 0
        i.roi.height = 0
        i.roi.width = 0
        i.roi.do_rectify = False

        # from left hand calibration
        i.K = [361.65757169, 0., 330.72019198, 0., 362.05007715, 209.74890311, 0.0, 0.0, 1.0]
        i.P = [361.65757169, 0.0, 330.72019198, 0.0, 0.0, 362.05007715, 209.74890311, 0.0, 0.0, 0.0, 1.0, 0.0]

        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            i.header.stamp = rospy.Time.now()
            # rospy.loginfo(i)
            self.pub.publish(i)

        rospy.spin()

if __name__ == '__main__':
    p = Publisher()