#!/usr/bin/env python

from config import *
import numpy as np
import rospy
from geometry_msgs.msg import *
from gatlin.msg import *
from gatlin.srv import *
import tf
from tf.msg import *

# FIXED_FRAME = "base"
FIXED_FRAME = "global_map"

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

    def get_transform(self, parent, child):
        if not (parent,child) in self.dts:
            self.dts[(parent,child)] = DynamicTransform(parent,child)

        tf = self.dts[(parent,child)].get_transform()
        # rospy.logerr(tf)

        return tf


class Frame:
    def __init__(self, transformer, max_estimates, target_frame, type, color="", id=""):
        self.transformer = transformer
        self.target_frame = target_frame # the target frame to position
        self.type = type
        self.id = id
        self.color = color

        self.ts = None # filtered position output from all inputs
        self.set = False

        self.twist = Twist() # use for velocity maybe
        self.obj_links = {}
        self.estimates = []
        self.max_estimates = max_estimates

    def add_obj_link(self, obj_color, obj_id, ts):
        self.obj_links["%s_%s" % (obj_color, obj_id)] = ts

    def get_fixed_to_target(self, obj, transformer):
        ts = self.obj_links["%s_%s" % (obj.color, obj.id)]

        target_frame = self.id
        attached_frame = ts.header.frame_id

        attached_to_obj_t = ts.transform
        attached_to_obj_matrix = transform_to_matrix(attached_to_obj_t)

        if attached_frame != target_frame:
            target_to_attached_t = transformer.get_transform(target_frame, attached_frame)
            target_to_attached_matrix = transform_to_matrix(target_to_attached_t)

            target_to_obj_matrix = np.dot(target_to_attached_matrix, attached_to_obj_matrix)

        else:
            target_to_obj_matrix = attached_to_obj_matrix

        fixed_to_obj_t = transform_from_pose(ar.pose.pose)
        fixed_to_obj_matrix = transform_to_matrix(fixed_to_obj_t)
        
        obj_to_target_matrix = np.linalg.inv(target_to_obj_matrix)
        fixed_to_target_matrix = np.dot(fixed_to_obj_matrix, obj_to_target_matrix)
        fixed_to_target_t = transform_from_matrix(fixed_to_target_matrix)

        return fixed_to_target_t

    def update(self, ts):
        self.ts = ts
        self.set = True

    def update_transform(self, obj):
        if "%s_%s" % (obj.color, obj.id) in self.obj_links:

            t = self.get_fixed_to_target(obj, self.transformer)

            self.estimates.append(t)
            while len(self.estimates) > self.max_estimates:
                self.estimates.pop(0)

            if len(self.estimates) > 1:
                mean_transform = getAverageTransform(self.estimates)

                self.ts = TransformStamped()
                self.ts.header.frame_id = FIXED_FRAME
                self.ts.child_frame_id = self.id
                self.ts.transform = mean_transform
                self.set = True

class FM:
    def __init__(self):
        rospy.init_node('frame_maintainer')

        # take in all object and ar poses and
        rospy.Subscriber("/gatlin/ar_marker_list", ObjectList, self.ar_marker_list_cb, queue_size=3)
        rospy.Subscriber("/baxter_left/ar_marker_list", ObjectList, self.ar_marker_list_cb, queue_size=3)
        rospy.Subscriber("/baxter_right/ar_marker_list", ObjectList, self.ar_marker_list_cb, queue_size=3)
        
        self.ol_pub = rospy.Publisher("/server/ar_marker_list", ObjectList, queue_size=1)

        self.tfb = tf.TransformBroadcaster()

        # use them to update corresponding frames in the global map

        self.frames = []

        self.transformer = Transformer()

        baxter = Frame(self.transformer, 15, "baxter", "transform")
        baxter.add_obj_link("ar", "1",self.create_ts(
            "baxter", "ar_marker_1_expected",
            0,0,0, 0,0,0,1
        ))
        # initialize baxter in global_map
        baxter.update(self.create_ts(
            FIXED_FRAME, "baxter",
            1,0,0, 0,0,0,1
        ))
        self.frames.append(baxter)
        
        gatlin = Frame(self.transformer, 15, "gatlin", "transform")
        # initialize gatlin in global_map
        gatlin.update(self.create_ts(
            FIXED_FRAME, "gatlin",
            0,0,0, 0,0,0,1
        ))
        self.frames.append(gatlin)
        
        rospy.sleep(1)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.broadcast_transforms()
            rate.sleep()

        rospy.spin()

    def create_obj_frame(self, i, width):
        obj = Frame(self.transformer, 15, "ar_"+i, "object", color="ar", id=i)
        obj.add_obj_link("ar",i,self.create_ts(
            "ar_"+i, "ar_marker_%s_expected"%i,
            0,0,0, 0,0,0,1
            # -0.007, -0.037, -0.083,
            # -7.07106781e-01, -7.07106781e-01, 0.0, 0.0
        ))
        self.frames.append(obj)

    def broadcast_transforms(self):
        ol = ObjectList()
        for f in self.frames:
            if f.set:
                if f.type == "transform":
                    f.ts.header.stamp = rospy.Time.now()
                    f.ts.child_frame_id = f.target_frame
                    # rospy.logerr(f.ts)
                    self.tfb.sendTransformMessage(f.ts)
                    rospy.loginfo("Published Global Transform")
                elif f.type == "object":
                    o = Object()
                    o.id = f.id
                    o.color = f.color
                    o.pose = ts_to_ps(f.ts)
                    # rospy.logerr(o)
                    ol.objects.append(o)

        if len(ol.objects) > 0:
            # rospy.logerr(len(ol.objects))
            rospy.loginfo(ol)
            # rospy.loginfo("Published Global ObjectList")
            self.ol_pub.publish(ol)

    def create_ts(self, parent, child, tx,ty,tz, rx,ry,rz,rw):
        ts = TransformStamped()
        ts.transform.translation = Vector3(tx,ty,tz)
        ts.transform.rotation = Quaternion(rx,ry,rz,rw)

        ts.child_frame_id = child
        ts.header.frame_id = parent

        ts.header.stamp = rospy.Time.now()       
        return ts

    def ar_marker_list_cb(self, ol):
        # rospy.logerr(ol)
        for obj in ol.objects:
            if obj.color == "ar":
                for f in self.frames:
                    f.update_transform(obj)

if __name__ == '__main__':
    fm = FM()
