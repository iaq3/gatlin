#!/usr/bin/env python

from config import *
import numpy as np
import rospy
from geometry_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *
import tf
from tf.msg import *

FIXED_FRAME = "global_map"
transformer = Transformer()        

class DynamicTransform:
    def __init__(self, parent, child):
        self.sub = rospy.Subscriber('%s_in_%s' % (child, parent), PoseStamped, self.poseCb, queue_size=1)
        self.parent = parent
        self.child = child
        self.trans = None

    def poseCb(self, ps):
        self.trans = transform_from_pose(ps.pose)

    def get_transform(self):
        if self.trans == None:
            rospy.logerr("no transform %s -> %s yet." % (self.parent, self.child))
        return self.trans


class Transformer:
    def __init__(self):
        self.tfl = tf.TransformListener()
        self.dts = {} # dynamic transforms

    def get_transform(parent, child):
        if not (parent,child) in self.dts:
            self.dts[(parent,child)] = DynamicTransform(parent,child)

        tf = self.dts[(parent,child)].get_transform()
        rospy.logerr(tf)

        return tf

class Frame:
    def __init__(self, ID):
        self.id = ID # the target frame to position

        self.ts = None # filtered position output from all inputs
        self.set = False

        self.twist = Twist() # use for velocity maybe
        self.ar_links = {}
        self.max_estimates = 20

    def add_ar_link(self, ar_id, ts):
        self.ar_links[ar_id] = ts

    def get_fixed_to_target(self, ar):
        global transformer
        ts = self.ar_links[ar.id]

        target_frame = self.id
        attached_frame = ts.header.frame_id

        attached_to_ar_t = ts.transform
        attached_to_ar_matrix = transform_to_matrix(attached_to_ar_t)

        if attached_frame != target_frame:
            target_to_attached_t = transformer.get_transform(target_frame, attached_frame)
            target_to_attached_matrix = transform_to_matrix(target_to_attached_t)

            target_to_ar_matrix = np.dot(target_to_attached_matrix, attached_to_ar_matrix)

        else:
            target_to_ar_matrix = attached_to_ar_matrix

        fixed_to_ar_t = transform_from_pose(ar.pose.pose)
        fixed_to_ar_matrix = transform_to_matrix(fixed_to_ar_t)
        
        ar_to_target_matrix = np.linalg.inv(target_to_ar_matrix)
        fixed_to_target_matrix = np.dot(fixed_to_ar_matrix, ar_to_target_matrix)
        fixed_to_target_t = transform_from_matrix(fixed_to_target_matrix)

        return fixed_to_target_t

    def update_transform(self, ar):
        if ar.id in self.ar_links:

            t = self.get_fixed_to_target(ar)

            self.estimates.append(t)
            while len(self.estimates) > self.max_estimates:
                self.estimates.pop(0)

            mean_transform = getAverageTransform(self.estimates)

            self.ts = TransformStamped()
            self.ts.header.frame_id = FIXED_FRAME
            self.ts.child_frame_id = ID
            self.ts.transform = mean_transform
            self.set = True

class FM:
    def __init__(self):
        rospy.init_node('frame_maintainer')

        # take in all object and ar poses and
        # rospy.Subscriber("/gatlin/ar_marker_list", ObjectList, self.ar_marker_list_cb, queue_size=3)
        rospy.Subscriber("/baxter_left/ar_marker_list", ObjectList, self.ar_marker_list_cb, queue_size=3)
        rospy.Subscriber("/baxter_right/ar_marker_list", ObjectList, self.ar_marker_list_cb, queue_size=3)

        self.tfb = tf.TransformBroadcaster()

        # use them to update corresponding frames in the global map

        self.frames = []

        baxter = Frame("baxter_link")
        baxter.add_ar_link("1",create_ts(
            "left_hand", "ar_marker_1_expected",
            0,0,0, 0,0,0,1
        ))
        baxter.add_ar_link("2",create_ts(
            "base", "ar_marker_2_expected",
            0,0,0, 0,0,0,1
        ))
        self.frames.append(baxter)

        obj_width = .04
        # get max block size that will fit in gatlin's gripper
        self.create_obj_frame("3", obj_width)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.broadcast_transforms()

        rospy.spin()

    def create_obj_frame(self, i, width):
        obj = Frame("obj"+i)
        obj.add_ar_link(i,create_ts(
            "obj"+i,
            0,0,-width/2, 0,0,0,1
        ))
        self.frames.append(obj)

    def broadcast_transforms(self):
        for f in self.frames:
            if f.set:
                f.ts.header.stamp = rospy.Time.now()
                self.tfb.sendTransformMessage(self.ts)
                rospy.logerr(self.ts)

    def create_ts(parent, child, tx,ty,tz, rx,ry,rz,rw):
        ts = TransformStamped()
        ts.transform.translation = Vector3(tx,ty,tz)
        ts.transform.rotation = Quaternion(rx,ry,rz,rw)

        ts.child_frame_id = child
        ts.header.frame_id = parent

        ts.header.stamp = rospy.Time.now()       


    def ar_marker_list_cb(self, ol):
        for obj in ol.objects:
            if obj.color == "ar":
                for f in self.frames:
                    f.update_transform(obj)




if __name__ == '__main__':
    fm = FM()