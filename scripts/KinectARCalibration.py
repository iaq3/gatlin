#!/usr/bin/env python
import cv2
import tf
from tf.msg import *
from config import *
import numpy as np
import math


class KinectARCalibration:
    def __init__(self):
        rospy.init_node('Kinect_AR_Calibration')

        self.tfl = tf.TransformListener()
        self.tfb = tf.TransformBroadcaster()
        self.FIXED_FRAME = "/base"
        self.RGB_FRAME = "/camera_rgb_optical_frame"
        self.CAMERA_FRAME = "/camera_link"
        self.AR_ID = 2
        self.ESTIMATED_AR_FRAME = "/ar_marker_%d" % self.AR_ID
        self.EXPECTED_AR_FRAME = "/ar_marker_%d_expected" % self.AR_ID

        self.tf_sub = rospy.Subscriber("/tf", tfMessage, self.tf_callback, queue_size=1)

        self.transforms = []
        self.num_transforms = 100

        # new idea:
        # transform from expected to camera with inverse of camera to estimated
        # convert to base frame and average removing outliers
        # broadcast camera tf

        rospy.spin()

    def transform_to_matrix(self, transform):
        t = deepcopy(transform.translation)
        t = [t.x,t.y,t.z]
        T = translation_matrix(t)
        q = deepcopy(transform.rotation)
        q = [q.x,q.y,q.z,q.w]
        R = quaternion_matrix(q)
        return np.dot(T,R)

    def transform_from_matrix(self, matrix):
        trans = Transform()
        q = quaternion_from_matrix(matrix)
        trans.rotation = Quaternion(q[0],q[1],q[2],q[3])
        t = translation_from_matrix(matrix)
        trans.translation = Vector3(t[0],t[1],t[2])
        return trans

    def inverse_transform(self, transform):
        matrix = self.transform_to_matrix(transform)
        inv_matrix = np.linalg.inv(matrix)
        return self.transform_from_matrix(inv_matrix)

    def get_transform(self, parent, child):
        self.tfl.waitForTransform(child, parent, rospy.Time(0), rospy.Duration(4))
        (T,R) = self.tfl.lookupTransform(parent, child, rospy.Time(0))
        tf = Transform()
        tf.rotation = Quaternion(R[0],R[1],R[2],R[3])
        tf.translation = Vector3(T[0],T[1],T[2])
        return tf

    def getAverageTransform(self):
        # get mean translation and rotation
        translations = np.zeros((0,3))
        rotations = np.zeros((0,4))
        for trans in self.transforms:
            t = deepcopy(trans.translation)
            translations = np.append(translations, [[t.x,t.y,t.z]], axis=0)

            q = deepcopy(trans.rotation)
            rotations = np.append(rotations, [[q.x,q.y,q.z,q.w]], axis=0)

        mean_translation = np.mean(translations, axis=0)
        mean_rotation = np.mean(rotations, axis=0)

        error_translations = translations - mean_translation
        norm_error_translations = np.linalg.norm(error_translations, axis=1)
        mean_norm_error_translation = np.mean(norm_error_translations)
        std_dev_norm_error_translation = np.std(norm_error_translations)

        error_rotations = rotations - mean_rotation
        norm_error_rotations = np.linalg.norm(error_rotations, axis=1)
        mean_norm_error_rotation = np.mean(norm_error_rotations)
        std_dev_norm_error_rotation = np.std(norm_error_rotations)

        # use mean and std to filter inliers
        inlier_tfs = []
        translation_threshold = 1.5
        rotation_threshold = 1.5
        for trans in self.transforms:
            t = deepcopy(trans.translation)
            t = np.array([t.x,t.y,t.z])

            q = deepcopy(trans.rotation)
            q = np.array([q.x,q.y,q.z,q.w])
            
            norm_error_translation = np.linalg.norm(t - mean_translation)
            translation_z_score = (norm_error_translation - mean_norm_error_translation) / std_dev_norm_error_translation
            if math.isnan(translation_z_score): translation_z_score = 0
            inlier_translation = abs(translation_z_score) < translation_threshold

            norm_error_rotation = np.linalg.norm(q - mean_rotation)
            rotation_z_score = (norm_error_rotation - mean_norm_error_rotation) / std_dev_norm_error_rotation
            if math.isnan(rotation_z_score): rotation_z_score = 0
            inlier_rotation = abs(rotation_z_score) < rotation_threshold

            if inlier_translation and inlier_rotation:
                inlier_tfs.append(trans)
            else:
                rospy.logerr("outlier")
                rospy.logerr(translation_z_score)
                rospy.logerr(rotation_z_score)
                rospy.logerr(trans)

        # get the mean again with the inliers
        translations = np.zeros((0,3))
        rotations = np.zeros((0,4))
        for trans in inlier_tfs:
            t = deepcopy(trans.translation)
            translations = np.append(translations, [[t.x,t.y,t.z]], axis=0)

            q = deepcopy(trans.rotation)
            rotations = np.append(rotations, [[q.x,q.y,q.z,q.w]], axis=0)

        mean_translation = np.mean(translations, axis=0)
        mean_rotation = np.mean(rotations, axis=0)

        mean_transform = Transform()
        mean_transform.translation = Vector3(mean_translation[0],mean_translation[1],mean_translation[2])
        mean_transform.rotation = Quaternion(mean_rotation[0],mean_rotation[1],mean_rotation[2],mean_rotation[3])
        return mean_transform

    def tf_callback(self, tfmsg):
        for trans in tfmsg.transforms:
            if trans.child_frame_id == self.ESTIMATED_AR_FRAME:
                rgb_to_ar_t = deepcopy(trans.transform)
                rgb_to_ar_matrix = self.transform_to_matrix(rgb_to_ar_t)
                ar_to_rgb_matrix = np.linalg.inv(rgb_to_ar_matrix)

                fixed_to_ar_t = self.get_transform(self.FIXED_FRAME, self.EXPECTED_AR_FRAME)
                fixed_to_ar_matrix = self.transform_to_matrix(fixed_to_ar_t)

                rgb_to_camera_t = self.get_transform(self.RGB_FRAME, self.CAMERA_FRAME)
                rgb_to_camera_matrix = self.transform_to_matrix(rgb_to_camera_t)

                fixed_to_rgb_matrix = np.dot(fixed_to_ar_matrix, ar_to_rgb_matrix)
                fixed_to_camera_matrix = np.dot(fixed_to_rgb_matrix, rgb_to_camera_matrix)
                fixed_to_camera_t = self.transform_from_matrix(fixed_to_camera_matrix)

                self.transforms.append(fixed_to_camera_t)

                if len(self.transforms) >= self.num_transforms:
                    mean_transform = self.getAverageTransform()
                    rospy.logerr(mean_transform)
                    self.transforms = []

    def get_pose(self, child, parent):
        try:
            ps = PoseStamped()
            ps.pose.orientation = Quaternion(0, 0, 0, 1)
            ps.pose.position = Point(0, 0, 0)
            ps.header.frame_id = child
            ps.header.stamp = rospy.Time(0)
            self.tfl.waitForTransform(child, parent, ps.header.stamp, rospy.Duration(4))
            return self.tfl.transformPose(parent, ps)
        except Exception as e:
            rospy.logerr(e)
            return None

if __name__ == '__main__':
    try:
        k = KinectARCalibration()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
