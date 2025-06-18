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

from pyzbar import pyzbar

import rospy

class CameraPreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False
		
		self.fontFace = cv2.FONT_HERSHEY_SIMPLEX
		self.fontScale = 2
		self.color = (0, 0, 255)
		self.colorPose = (0, 0, 255)
		self.colorIMU = (255, 0, 255)
		self.thickness = 2
		self.lineType = cv2.LINE_AA
		self.bottomLeftOrigin = False # if True (text upside down)

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
		
		# Status
		cv2.putText(self.cv_image_clone, 
			"Battery: %.4f" % (self.battery), 
			(30, 30), 
			self.fontFace, 
			self.fontScale, 
			self.color, 
			self.thickness, 
			self.lineType, 
			self.bottomLeftOrigin)
		
		cv2.imshow("Camera Preview", self.cv_image_clone)
		cv2.waitKey(1)

	def cbQRMission(self):
		# Converting to grayscale
		cv_image_gray = cv2.cvtColor(self.cv_image_clone, cv2.COLOR_BGR2GRAY)

		# QR Detection
		results = pyzbar.decode(cv_image_gray)
		
		# loop over the detected barcodes
		for result in results:
		
			# extract the bounding box location of the barcode and draw the
			# bounding box surrounding the barcode on the image
			(x, y, w, h) = result.rect
#			cv2.rectangle(self.cv_image_clone, (x, y), (x + w, y + h), (0, 0, 255), 2)
		
			# the barcode data is a bytes object so if we want to draw it on
			# our output image we need to convert it to a string first
			barcodeData = result.data.decode("utf-8")
#			barcodeType = result.type
		
#			# draw the barcode data and barcode type on the image
#			text = "{} ({})".format(barcodeData, barcodeType)
#			cv2.putText(self.cv_image_clone, text, (x, y - 10), self.fontFace,
#				self.fontScale, self.color, self.thickness)

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
