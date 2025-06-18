#!/usr/bin/env python

################################################################################
## {Description}: Preview an Image from Bebop Camera Stream
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

"""
Image published (CompressedImage) from tello originally size of 960x720 pixels
We will try to resize it using imutils.resize (with aspect ratio) to width = 320
and then republish it as Image
"""

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import numpy as np
import imutils
import random

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  # For battery percentage
from geometry_msgs.msg import Twist

import rospy

class CameraPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False

		rospy.logwarn("Camera Preview Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.telloImage_topic = "/bebop/image_raw/compressed"
		self.telloImage_sub = rospy.Subscriber(
						self.telloImage_topic, 
						CompressedImage, 
						self.cbImage
						)
						
		self.bebopBattery_topic = "/bebop/states/common/CommonState/BatteryStateChanged"
		self.bebopBattery_sub = rospy.Subscriber(
						self.bebopBattery_topic, 
						CommonCommonStateBatteryStateChanged, 
						self.cbBattery
						)
						
		self.bebopCamdown_topic = "/bebop/camera_control"
		self.bebopCamdown_pub = rospy.Publisher(
						self.bebopCamdown_topic, 
						Twist, 
						queue_size=1
						)
						
		# Allow up to one second to connection
		rospy.sleep(1)

	# Get the battery percentage
	def cbBattery(self, msg):
		self.battery = msg.percent

	# Tilt down the camera in 90 degrees
	def cam_down(self):
		cam = Twist()
		cam.angular.y = -90.0
		self.bebopCamdown_pub.publish(cam)
		
	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			# direct conversion to cv2
			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
								msg, 
								"bgr8"
								)
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False
			
	# Show the output frame
	def cbShowImage(self):

		self.cv_image_clone = imutils.resize(
					self.cv_image.copy(), 
					width=320
					)
					
		self.cv_image_clone = self.cv_image
		
		fontFace = cv2.FONT_HERSHEY_PLAIN
		fontScale = 2
		color = (255, 255, 255)
		colorPose = (0, 0, 255)
		colorIMU = (255, 0, 255)
		thickness = 2
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

#		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

		# Status
		cv2.putText(self.cv_image_clone, 
			"Battery: %.4f" % (self.battery), 
			(30, 30), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)
		
		cv2.imshow("Camera Preview", self.cv_image_clone)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("Camera Preview Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_preview', anonymous=False)
	camera = CameraPreview()
	
#	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
#		r.sleep()
