#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
from cs4752_proj3.srv import *
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
from tf.transformations import *
from config import *
from copy import deepcopy


#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython



class hand_camera_calibration:

	def __init__(self):
		print "initializing hand_camera_calibration"

		self.bridge = CvBridge()

		self.limb_name = 'left'
		self.topic = "/cameras/"+self.limb_name+"_hand_camera/image"
		# topic = "/camera/rgb/image_rect_color"
		self.image_sub = rospy.Subscriber(self.topic,Image,self.imagecallback, queue_size=1)

		self.images = []

		cv2.startWindowThread()
		cv2.namedWindow('hand_camera_calibration')

		self.mtx = []

	def imagecallback(self,data):

		print "got image"

		try:
			img = self.bridge.imgmsg_to_cv2(data, "passthrough")#rgba8
		except CvBridgeError, e:
			print e

		self.images.append(img)

		if(len(self.images) == 10):
			self.image_sub.unregister()
			mtx = self.calibrate()
			self.mtx.append(mtx)
			self.images = []
			if len(self.mtx) < 5:
				self.image_sub = rospy.Subscriber(self.topic,Image,self.imagecallback, queue_size=1)
			else:
				answer = np.mean(self.mtx, axis=0)
				rospy.logerr(answer)
				# rospy.logerr(self.mtx)
				cv2.destroyAllWindows()

	def calibrate(self):
		print "proccessing images"

		# termination criteria
		criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		actual_size = (9,7)
		size = (actual_size[0]-1,actual_size[1]-1)

		# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
		objp = np.zeros((size[1]*size[0],3), np.float32)
		objp[:,:2] = np.mgrid[0:size[0],0:size[1]].T.reshape(-1,2)

		# Arrays to store object points and image points from all the images.
		objpoints = [] # 3d point in real world space
		imgpoints = [] # 2d points in image plane.

		# images = glob.glob('*.jpg')

		for img in self.images:
			# img = cv2.imread(fname)
			gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

			# cv2.imshow('img',gray)
			# cv2.waitKey(500)

			# Find the chess board corners
			ret, corners = cv2.findChessboardCorners(gray, size, None)

			# If found, add object points, image points (after refining them)
			if ret == True:
				objpoints.append(objp)

				# corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
				# print corners2
				imgpoints.append(corners)

				# Draw and display the corners
				cv2.drawChessboardCorners(img, size, corners, ret)
				cv2.imshow('hand_camera_calibration',img)
				cv2.waitKey(200)

		print "done"


		# objpoints = np.array(objpoints).astype('float32')
		# imgpoints = np.array(imgpoints).astype('float32')

		print "calibrating"
		objpoints = np.array(objpoints,'float32')
		imgpoints = np.array(imgpoints,'float32')
		ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

		fovx, fovy, focalLength, principalPoint, aspectRatio = cv2.calibrationMatrixValues(mtx, gray.shape[::-1], 1.0, 1.0)

		# print "mtx"
		# print ""
		# print mtx
		# print ""
		# print ""
		# print "fovx"
		# print fovx
		# print "fovy"
		# print fovy
		# print "focalLength"
		# print focalLength

		# fovx
		# 179.641902544
		# fovy
		# 179.427046979

		return mtx



def main(args):
  
  rospy.init_node('hand_camera_calibration', anonymous=True)
  print "initialized node hand_camera_calibration"
  ic = hand_camera_calibration()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
