#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import time
from std_msgs.msg import String, Header
from geometry_msgs.msg import Pose, Quaternion, Point, PoseArray, PoseStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import *
import numpy as np
from copy import deepcopy
import math
import tf
from tf.transformations import *
#from config import *
import sys, os

def PointDistance (p1, p2) :
	return ((p1.x-p2.x)** 2 + (p1.y - p2.y)** 2 + (p1.z - p2.z)**2)**.5

class LiveFilter:
	def __init__(self) :
		self.trackable = False
		self.lastPosition = Point()
		self.lastCall = 0.0
		self.timeStable = 0
		self.speedMax = .5
		self.minTimeStable = .3
		#outputPosition = Point
	def updateFilter(self, newpoint) :
		
		deltaTime = rospy.Time.now().to_sec() - self.lastCall
		dist = PointDistance(newpoint, self.lastPosition)
		speed = dist/deltaTime
		outputPosition = None

		if (deltaTime > self.minTimeStable) :
			#object timeout
			self.trackable = False;
			self.timeStable = 0
			
		elif (self.trackable) :
			if (speed > self.speedMax) :
				#no longer trackable object
				self.trackable = False
				self.timeStable = 0
			else :
				#still trackable object
				outputPosition = newpoint
		else :
			if (speed < self.speedMax) :
				self.timeStable += deltaTime
				if (self.timeStable > self.minTimeStable) :
					#the object is in bound, not tracked, but has been stable to be tracked, it is now trackable
					self.trackable = True
					outputPosition = newpoint
			else :
				#the time is in bound, the object has not been trackable and its moving too fast to track
				self.timeStable = 0
		self.lastCall = rospy.Time.now().to_sec()
		self.lastPosition = newpoint

		return outputPosition
		#filtered position, publish outputPosition

#takes in a list of filters and returns the first in the distance, or the longest to be updated
def getClosestIndex(liveFilterArray, dist, newpoint) : #, pastPoints
	count = 0
	shortestDistance = 100000
	shortestDistanceIndex = -1
	for f in liveFilterArray :
		distance_to = PointDistance(f.lastPosition, newpoint)
		
		if distance_to < shortestDistance :
			shortestDistance = distance_to
			shortestDistanceIndex = count
		count += 1
	#shortestDistance < dist and 
	if shortestDistanceIndex >-1 :
		return (shortestDistanceIndex, shortestDistance)
	return (-1, 100)



class HSVMask:
	def __init__(self, color, shape_name, camera, mask, num_blobs=1, calibrated=True):
		self.color = color
		self.shape_name = shape_name
		self.camera = camera

		self.calibrated = calibrated # set to False when you need to calibrate
		self.param = "H"
		self.prompt = True
		self.m = mask
		self.num_blobs = num_blobs

		# self.shape = cv2.imread(shape, 0)
		# cv2.imshow("test", self.shape)

		self.window_name = '%s %s vision' % (color, camera)
		self.filters = []
		self.base_pubs = []
		for i in range(0, num_blobs):
			base_pub = rospy.Publisher("/%s_%s_%d_pose" % (color, camera, i), Pose, queue_size=1)
			self.base_pubs.append(base_pub)
			new_filter = LiveFilter()
			self.filters.append(new_filter)

		# self.blue_screen_pub = rospy.Publisher("/found_blue_%s" % self.vision_type, ScreenObj, queue_size=1)

		print "os.getcwd()"
		print os.getcwd()

		self.shape = cv2.imread('../ros_ws/src/gatlin/img/%s.jpg' % shape_name, 0)
		# cv2.imshow("test", self.shape)
		# cv2.waitKey(0)

		cv2.startWindowThread()
		cv2.namedWindow(self.window_name)


	def changeMask(self, param, arg, inc):
		limit = {}
		limit["H"] = {}
		limit["S"] = {}
		limit["V"] = {}
		limit["D"] = {}
		limit["H"]["min"] = 0
		limit["S"]["min"] = 0
		limit["V"]["min"] = 0
		limit["D"]["min"] = -1.0
		limit["H"]["max"] = 180
		limit["S"]["max"] = 255
		limit["V"]["max"] = 255
		limit["D"]["max"] = 10000.0

		self.m[param][arg] += inc

		if arg == "min":
			if self.m[param][arg] < limit[param][arg]:
				self.m[param][arg] = limit[param][arg]
			elif self.m[param][arg] > self.m[param]["max"]:
				self.m[param][arg] = self.m[param]["max"]
		elif arg == "max":
			if self.m[param][arg] > limit[param][arg]:
				self.m[param][arg] = limit[param][arg]
			elif self.m[param][arg] < self.m[param]["min"]:
				self.m[param][arg] = self.m[param]["min"]
		
		print "%s %s" % (param, arg)
		print self.m[param][arg]

	def calibrate(self):
		if self.prompt :
			print "Calibrating %s %s HSV Mask" % (self.color, self.camera)
			print "Click on the image, then..."
			print "use H, S, V, and D to switch the changing param"
			print ""
			print "use U and J to change min"
			print "use O and L to change max"
			print ""
			print "press any other key to get a new frame"
			print "Press space whena done"
			self.prompt = False

		key = cv2.waitKey(0) & 0xFF

		inc =2
		if self.param == "D":
			inc = 10

		if key == ord("u"):
			self.changeMask(self.param, "min", inc)

		if key == ord("j"):
			self.changeMask(self.param, "min", -inc)
		
		if key == ord("o"):
			self.changeMask(self.param, "max", inc)
		
		if key == ord("l"):
			self.changeMask(self.param, "max", -inc)


		if key == ord("h"):
			print "Now altering H"
			self.param = "H"

		if key == ord("s"):
			print "Now altering S"
			self.param = "S"
		
		if key == ord("v"):
			print "Now altering V"
			self.param = "V"

		if key == ord("d"):
			print "Now altering D"
			self.param = "D"


		if key == ord(" "):
			self.calibrated = True
			print "############## Calibrated %s %s Mask ##############" % (self.color, self.camera)
			print self.m
			print "#############################################"

class Vision:

	def __init__(self):

		self.vision_type = "kinect"#rospy.get_namespace()[1:-1]

		rospy.init_node("%s_vision" % self.vision_type)

		print "Initializing %s Vision" % self.vision_type

		self.num_blocks = 3#rospy.get_param("/num_blocks")

		rospy.set_param("/camera/driver/depth_registration", True)

		self.lastImageTime = time.time()
		self.imageWaitTime = .01



		self.masks = []

		#self.pink_kinect_mask = HSVMask(
		#	"pink",
		#	"circle",
		#	"kinect",
		#	{'H': {'max': 178, 'min': 160}, 'S': {'max': 255.0, 'min': 154.0}, 'D': {'max': 2500, 'min': 100}, 'V': {'max': 255.0, 'min': 70.0}},
		#	calibrated=True
		#)
		#self.masks.append(self.pink_kinect_mask)

		# self.blue_kinect_mask = HSVMask(
		# 	"blue",
		# 	"square",
		# 	"kinect",
		# 	{'H': {'max': 140.0, 'min': 100.0}, 'S': {'max': 180.0, 'min': 83.0}, 'D': {'max': 1.7, 'min': 1.4}, 'V': {'max': 255, 'min': 115.0}},
		# 	calibrated=True,
		# 	num_blobs=self.num_blocks
		# )
		# self.masks.append(self.blue_kinect_mask)

		self.green_kinect_mask = HSVMask(
			"green",
			"circle",
			"kinect",
			{'H': {'max': 68, 'min': 38}, 'S': {'max': 255, 'min': 70}, 'D': {'max': 2000, 'min': 0}, 'V': {'max': 231.0, 'min': 40.0}},
			calibrated=True, num_blobs = 1
		)
		self.masks.append(self.green_kinect_mask)
		


		self.transform_listener = tf.TransformListener()
		self.bridge = CvBridge()	

		if self.vision_type == "kinect":
			self.rgb_topic = "/camera/rgb/image_rect_color_throttled"


			#self.depth_topic = "/camera/depth_registered/hw_registered/image_rect_raw/compressed"
			self.depth_topic = "/camera/depth_registered/hw_registered/image_rect_raw_throttled"
			#rostopic list "/camera/depth/image_rect/compressed"
			#self.depth_topic = "/camera/depth_registered/hw_registered/image_rect_raw"
			#self.depth_topic = "/camera/depth_registered/sw_registered/image_rect"
			self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
			print "subscribed to %s" % self.depth_topic

		elif self.vision_type == "hand":
			self.rgb_topic = "/cameras/"+self.limb_name+"_hand_camera/image"

		self.rgb_sub = rospy.Subscriber(self.rgb_topic,Image,self.rgb_callback, queue_size=1)
		print "subscribed to %s" % self.rgb_topic

		self.rgb_image = None
		self.depth_image = None

		
		
		
		self.rate = rospy.Rate(30)
		self.pixel_radius = 10#2.1539 #radius in pixels at 1 meter of orange ball
		
	

		print "done initializing"

		rospy.spin()

	def rgb_callback(self,data):
		try:
			self.rgb_image = self.bridge.imgmsg_to_cv2(data, "passthrough")#rgba8 "bgr8"
		except CvBridgeError, e:
			print e

		for mask in self.masks:
			# print "self.find_project_publish(%s)" % mask.color
			self.find_project_publish(mask)

	#given an hsv_mask, finds the number of blobs, filters, and publishes if it's consistent
	def find_project_publish(self, hsv_mask):
		objs = self.findBlobsofHue(hsv_mask, hsv_mask.num_blobs , self.rgb_image)
		#print "-------------"
		
		object_points = []
		for bi in objs:
			# bi = objs[0]
			radius = bi[2]
			if self.depth_image != None :
				distance = self.depth_image[int(bi[1]), int(bi[0])]
				if math.isnan(distance) :
					distance = self.pixel_radius / radius
					print "NAN returning instead of using bad depth"
					return
				obj_pose = self.project((bi[0], bi[1]), distance, self.rgb_image.shape[1], self.rgb_image.shape[0])
			else :
				distance = self.pixel_radius / radius
				obj_pose = self.project((bi[0], bi[1]), distance, self.rgb_imageCREATE .shape[1], self.rgb_image.shape[0])
			if obj_pose != None :
				obj_pose.position.x = obj_pose.position.x / 1000
				obj_pose.position.y = obj_pose.position.y / 1000
				obj_pose.position.z = obj_pose.position.z / 1000
				(index,dist) = getClosestIndex( hsv_mask.filters, 10, obj_pose.position)
				#finds closest object-filter pair for each object
				#print index, dist
				object_points.append((obj_pose, index, dist))
				
		#pair of index of object for that filter and distance
		hsv_counts = []	
		
		for i in xrange(hsv_mask.num_blobs) :
			hsv_counts.append([-1, 100, []])
		c = 0
		#finds closest object-filter pair for the filter
		for (obj_pose, closest_filter , dist) in object_points :
			if closest_filter > -1 :
				if (hsv_counts[closest_filter][1] > dist) :
					#print "closer obj found"
					hsv_counts[closest_filter][0] = c
					hsv_counts[closest_filter][1] = dist
				hsv_counts[closest_filter][2].append(c)
			c += 1
		
		freeFilters = []
		freePoints = []
		#publishes best filter-object pairs and collects the bad ones
		for i in xrange(len(hsv_mask.filters)) :
			
			obj_index = hsv_counts[i][0]
			if obj_index > -1 :
				bestPose = object_points[obj_index][0]
				output_object = hsv_mask.filters[i].updateFilter(bestPose.position)
				if (output_object) :
					try :
						hsv_mask.base_pubs[i].publish(bestPose)
					except CvBridgeError, e:
						print e
				for point_index in hsv_counts[i][2] :
					if obj_index != point_index :
						freePoints.append(point_index)
			else :
				freeFilters.append(i)
		
		#iterates throught the bad ones so they are in the system and will become good ones		
		c = 0
		while c < len(freeFilters) and c < len(freePoints):
			hsv_mask.filters[freeFilters[c]].updateFilter(object_points[freePoints[c]][0].position)
			c += 1
		#print "recombined ",c

	def depth_callback(self,data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
		except CvBridgeError, e:
			print e

	def findBlobsofHue(self, hsv_mask, num_blobs, rgb_image) :
		if self.vision_type == 'kinect' and self.depth_image == None:
			print("Error: no depth image")
			return []

		colorLower = (
			hsv_mask.m["H"]["min"], 
			hsv_mask.m["S"]["min"], 
			hsv_mask.m["V"]["min"]
		)
		colorUpper = (
			hsv_mask.m["H"]["max"], 
			hsv_mask.m["S"]["max"], 
			hsv_mask.m["V"]["max"]
		)

		rgb_image = self.rgb_image.copy()

		if self.vision_type == "kinect" and self.depth_image != None:
			depth_image = self.depth_image.copy()
			mask = cv2.inRange(depth_image, hsv_mask.m["D"]["min"], hsv_mask.m["D"]["max"])
			rgb_image = cv2.bitwise_and(rgb_image, rgb_image, mask = mask)

		blurred = cv2.GaussianBlur(rgb_image, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, colorLower, colorUpper)

		res = cv2.bitwise_and(rgb_image,rgb_image, mask = mask)

		blobsFound = []
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

		# h = rgb_image.shape[0]
		# w = rgb_image.shape[1]
		# center_img = np.array([w/2.,h/2.])
		# ret, thresh = cv2.threshold(hsv_mask.shape, 127, 255,0)
		# contours,hierarchy = cv2.findContours(thresh,2,1)
		# shape = contours[0]

		def get_score(c):
			try:
				# use shape of the obj to get similarity
				# similarity = 1
				# similarity = cv2.matchShapes(shape,c,1,0.0) + 1
				area = cv2.contourArea(c)
				# M = cv2.moments(c)
				# cx = int(M['m10']/M['m00'])
				# cy = int(M['m01']/M['m00'])
				# centroid = np.array([cx,cy])
				# center_error = np.linalg.norm(centroid - center_img)
				score = area * 1000
				# rospy.logerr(score)
				return score
			except:
				return 0

		def score_compare(c1, c2):
			tc1 = get_score(c1)
			tc2 = get_score(c2)
			minus = tc1 - tc2
			try :
				return int(minus)
			except e :
				return 0
			return 0

		cs = sorted(cnts, cmp=score_compare)[-num_blobs:]

		for c in cs:

			area = cv2.contourArea(c)
			# similarity = 0#cv2.matchShapes(shape,c,1,0.0) + 1

			if hsv_mask.shape_name == 'square':

				rect = cv2.minAreaRect(c)
				box = cv2.cv.BoxPoints(rect)
				box = np.int0(box)
				
				radius = math.sqrt(cv2.contourArea(c))
				if radius > 5 and area > 150 and similarity < 10.0:
					# get the centroid
					M = cv2.moments(c)
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])

					blobsFound.append([cx,cy,radius])
					cv2.drawContours(res,[box],0,(0,0,255),2)
			
			elif hsv_mask.shape_name == 'circle':
				((x, y), radius) = cv2.minEnclosingCircle(c)
				# rospy.logerr(area)

				
				# if radius > 3 and area > 80 and similarity < 2.0:
				if radius > 3 and area > 80:
					blobsFound.append([x,y,radius])
					cv2.circle(res, (int(x), int(y)), int(radius), (0,255,255), 2)

		cv2.imshow(hsv_mask.window_name, res)

		if not hsv_mask.calibrated:
			hsv_mask.calibrate()

		return blobsFound

	#creates an intrinsic camera matrix and uses the 
	#position and size of the ball to determine pose
	#relative to the camera, (using kinect specs)
	def project(self, point, distance, width, height) :
		#print point
		#print width
		#print height
		#print "radius"
		#print radius
		#print "Not using depth"

		xFOV = 63.38
		yFOV = 48.25
		cx = width /2
		cy = height /2
		fx = cx / np.tan((xFOV/2) * np.pi / 180)
		fy = cy / np.tan((yFOV/2) * np.pi / 180)
		
		toball = np.zeros(3)
		toball[0] = (point[0] - cx) / fx
		toball[1] = -(point[1] - cy) / fy
		toball[2] = 1
		toball = toball / np.linalg.norm(toball)
		
		toball = toball * distance

		pose = Pose()
		pose.position = Point(toball[0], -toball[1], toball[2])
		pose.orientation = Quaternion(0,0,0,1)
		return pose

if __name__ == '__main__':
	try:
		v = Vision()
	except KeyboardInterrupt:
		print "Shutting down"
	cv2.destroyAllWindows()
