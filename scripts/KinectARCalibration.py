#!/usr/bin/env python
import cv2
import rospy
import tf
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
from tf.msg import *
from tf.transformations import *

from config import *


class KinectARCalibration:
    def __init__(self):
        rospy.init_node('Kinect_AR_Calibration')

        self.tfl = tf.TransformListener()
        self.FIXED_FRAME = "/base"
        self.CAMERA_FRAME = "/camera_rgb_optical_frame"
        self.AR_ID = 2
        self.ESTIMATED_AR_FRAME = "/ar_marker_%d" % self.AR_ID
        self.EXPECTED_AR_FRAME = "/ar_marker_%d_expected" % self.AR_ID

        self.tf_sub = rospy.Subscriber("/tf", tfMessage, self.tf_callback, queue_size=1)

        # new idea:
        # transform from expected to camera with inverse of camera to estimated
        # convert to base frame and average removing outliers
        # broadcast camera tf

        # self.num_correspondences = 100

        # rate = rospy.Rate(30)
        # while not rospy.is_shutdown():
        # 	ps = self.get_pose(self.EXPECTED_AR_FRAME, self.FIXED_FRAME)
        # 	rospy.logerr(ps)

        rospy.spin()

    # TODO: get updated version from gatlin
    def tf_callback(self, tfmsg):
        for trans in tfmsg.transforms:
            if trans.child_frame_id == self.ESTIMATED_AR_FRAME:
                rospy.logerr(trans.transform.inverse)
            # add estimated and expected points in fixed frame
            # trans.inverse()

    # if trans.child_frame_id.startswith("ar_marker_"):
    # 			i = trans.child_frame_id.split("ar_marker_",1)[1]
    # 			if i == "255": return
    # 			rospy.logerr(i)
    # 			# rospy.logerr(trans.header.frame_id)
    # 			rospy.logerr(trans.child_frame_id)
    # 			rospy.logerr(trans.transform)

    # 			# TODO: republish for use later or
    # 			# maybe use the frame as the parent of baxter
    # 			# create static transform from ar_marker_# to baxter base link
    # 			# also filter with an EKF

    # 			# transform to fixed global map frame
    # 			try:
    # 				frame_id = self.ar_marker_dict[i]
    # 			except:
    # 				rospy.logerr("No attached frame for marker %s" % i)
    # 				continue

    # 			T = trans.transform.translation
    # 			R = trans.transform.rotation

    def get_pose(self, child, parent):
        try:
            ps = PoseStamped()
            ps.pose.orientation = Quaternion(0, 0, 0, 1)
            ps.pose.position = Point(0, 0, 0)
            ps.header.frame_id = child
            ps.header.stamp = rospy.Time(0)
            self.tfl.waitForTransform(child, parent, ps.header.stamp, rospy.Duration(4.0))
            return self.tfl.transformPose(parent, ps)
        except Exception as e:
            rospy.logerr(e)
            return None

    def get_kinect_transform(self, base_points, kinect_points):
        shape = (1, len(base_points), 3)
        source = np.zeros(shape, np.float32)
        target = np.zeros(shape, np.float32)

        count = 0
        for point_name in base_points:
            kinect_pt = list(kinect_points[point_name])
            source[0][count] = kinect_pt
            base_pt = list(base_points[point_name])
            # convert Point to list
            target[0][count] = base_pt
            count += 1
        # print source
        # print target

        retval, M, inliers = cv2.estimateAffine3D(source, target)
        M = np.append(M, [[0, 0, 0, 1]], axis=0)

        ts = TransformStamped()
        ts.header.frame_id = self.FIXED_FRAME
        ts.header.stamp = rospy.Time.now()
        ts.child_frame_id = self.CAMERA_FRAME
        ts.transform.translation = translation_from_matrix(M)
        ts.transform.rotation = quaternion_from_matrix(M)
        rospy.logerr(ts)

        return M


if __name__ == '__main__':
    try:
        k = KinectARCalibration()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
