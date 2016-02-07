#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Quaternion, Point
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython



class single_color_vision:

	def __init__(self):
		print "initializing single color vision object"
		self.ball_pub = rospy.Publisher("/ball_pose",Pose)
		#self.camerainfo_sub = rospy.Subscriber('camera/rgb/camera_info', CameraInfo, self.cameraIntrinsicsCB)
		#self.camera_matrix = None
		#cv2.namedWindow("Image window", 1)
		self.pixel_radius = 10#2.1539 #radius in pixels at 1 meter of orange ball
		self.lastImageTime = time.time()
		self.imageWaitTime = .1
		self.hueVal = 160
		#160 #pink

		#topic for the raw image/camera/depth_registered/image_raw
		#try camera/rgb/image_color/compressed for greater efficiency
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color",Image,self.imagecallback, queue_size=1)
		print "subscribed to /camera/rgb/image_rect_color"
		print "done initializing"

		cv2.startWindowThread()
		cv2.namedWindow('HSV_Mask')


		# for the image already converted, does not need cv bridge because image converter already did it
		#topic /image_converter/output_video
		#self.image_sub = rospy.Subscriber("/image_converter/output_video",Image,self.imagecallback)

	#def cameraIntrinsicsCB (self, data) :
	#	self.camera_matrix = data.k

	def imagecallback(self,data):
		#don't need to do every frame
		if self.lastImageTime > time.time() - self.imageWaitTime :
			return

		#print "got image"
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")#rgba8
		except CvBridgeError, e:
			print e
		#cv_image = data.image
		#print "Image shape"
		#print cv_image.shape
		
		#TODO fix depth image
		ball_pose = self.findBall(cv_image)
		try:
			if ball_pose != None :
				self.ball_pub.publish(ball_pose)
		except CvBridgeError, e:
			print e
		self.lastImageTime = time.time()



	#creates an intrinsic camera matrix and uses the 
	#position and size of the ball to determine pose
	#relative to the camera, (using kinect specs)
	def project(self, p, radius, width, height) :
		#print p
		#print width
		#print height
		#print "radius"
		#print radius

		xFOV = 63.38
		yFOV = 48.25
		cx = width /2
		cy = height /2
		fx = cx / np.tan((xFOV/2) * np.pi / 180)
		fy = cy / np.tan((yFOV/2) * np.pi / 180)
		
		toball = np.zeros(3)
		toball[0] = (p[0] - cx) / fx
		toball[1] = -(p[1] - cy) / fy
		toball[2] = 1
		toball = toball / np.linalg.norm(toball) #normalize so we can then multiply by distance
		distance = self.pixel_radius / radius
		toball = toball * distance

		pose = Pose()
		pose.position = Point(toball[0], toball[1], toball[2])
		pose.orientation = Quaternion(0,0,0,1)
		#print "FOUND ORANGE BALL!!!!"
		#print toball
		return pose

	#find a ball and return its transform relative to the camera
	#http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
	def findBall(self, rgbimage) :

		#self.hueVal = (self.hueVal + 2) % 180

		#print "Looking for Hue: "
		#print self.hueVal

		initialtime = time.time()
		#should be HSV values.
		hue_range = 4
		orangeLower = (self.hueVal-hue_range, 85, 6)
		orangeUpper =(self.hueVal+hue_range, 255, 255)

		# resize the frame, blur it, and convert it to the HSV
		# color space
		#frame = imutils.resize(frame, width=600)
		blurred = cv2.GaussianBlur(rgbimage, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, orangeLower, orangeUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)

		#grayimage = cv2.cvtColor(rgbimage, cv2.COLOR_RGB2GRAY)
		res = cv2.bitwise_and(rgbimage,rgbimage,mask = mask)

		
		# find contours in the mask and initialize the current
		# (x, y) center of the ball
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
		center = None
	 
		# only proceed if at least one contour was found
		if len(cnts) > 0:
			# find the largest contour in the mask, then use
			# it to compute the minimum enclosing circle and
			# centroid
			c = max(cnts, key=cv2.contourArea) #TODO LOOK AROUD HERE
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	 
			# only proceed if the radius meets a minimum size
			if radius > 9:

				pixelsInMask = 0
				for radr in xrange(0,int(radius)) :
					for radc in xrange(0, int(radius)) :
						#x,y?
						if x+radc >= 0 and x+radc < mask.shape[1] and y+radr >= 0 and y+radr < mask.shape[0] :
							if mask[y+radr,x+radc] == 255 :
								pixelsInMask += 1
		

				if pixelsInMask > radius * (.4 * radius) :

					#publish the location of the ball
					print "ball vision algorithm took"
					cv2.circle(res, (int(x), int(y)), int(radius), (0, 255, 255), 2)
					print time.time() - initialtime

					cv2.imshow('HSV_Mask',res)
					return self.project((x,y), radius, rgbimage.shape[1], rgbimage.shape[0])

		return None
	


def main(args):
  
  rospy.init_node('single_color_vision', anonymous=True)
  print "initialized node single color vision"
  ic = single_color_vision()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)