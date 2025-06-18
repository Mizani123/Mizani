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
import os
import select

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  # For battery percentage
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from pyzbar import pyzbar

import rospy

if os.name == 'nt':
	import msvcrt
else:
	import tty, termios

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
		
		self.QR1 = "N,N,N,W,4"
		self.QR2 = "N,N,E,S,0"
		self.QR3 = "N,E,E,S,0"
		self.QR4 = "W,S,S,S,1"
		self.QR5 = "W,E,S,S,2"
		self.QR6 = "W,N,E,W,3"
		
		self.QR1_Count = 0
		self.QR2_Count = 0
		self.QR3_Count = 0
		self.QR4_Count = 0
		self.QR5_Count = 0
		self.QR6_Count = 0
		
		self.QR_Previous = " "
		self.QR_Current = " "
		
		self.missionCount = 0
		
		self.takeoff = Empty()
		self.land = Empty()
		self.twist = Twist()
		
		self.QR_Detected = False
		
		self.cy = 0
		self.cx = 0
		
		self.takeoffed = 0
		self.landed = 1

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
						
		# Publish to Twist msg
		self.bebopTwist_topic = "/bebop/cmd_vel"
		self.bebopTwist_pub = rospy.Publisher(
					self.bebopTwist_topic, 
					Twist, 
					queue_size=10
					)

		# Publish to Empty msg
		self.bebopTakeoff_topic = "/bebop/takeoff"
		self.bebopTakeoff_pub = rospy.Publisher(
					self.bebopTakeoff_topic, 
					Empty, 
					queue_size=10)

		# Publish to Empty msg
		self.bebopLand_topic = "/bebop/land"
		self.bebopLand_pub = rospy.Publisher(
					self.bebopLand_topic, 
					Empty, 
					queue_size=10)
					
		# Allow up to one second to connection
		rospy.sleep(1)

	# Get the battery percentage
	def cbBattery(self, msg):
		self.battery = msg.percent

	# Tilt down the camera in 90 degrees
	def cbCamDown(self):
		cam = Twist()
		cam.angular.y = -90.0
		self.bebopCamdown_pub.publish(cam)
		
	# Tilt down the camera in 90 degrees
	def cbCamUp(self):
		cam = Twist()
		cam.angular.y = 90.0
		self.bebopCamdown_pub.publish(cam)

	def isTakeoff(self):
		time.sleep(3.5)
		self.start = time.time()
		self.takeoffed = 1
		self.landed = 0
		
	def isLand(self):
		self.landed = 1
		self.takeoffed = 0

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

#		self.cv_image_clone = imutils.resize(
#					self.cv_image.copy(), 
#					width=320
#					)
					
		self.cv_image_clone = self.cv_image.copy()
		
		# Status
#		cv2.putText(self.cv_image_clone, 
#			"Battery: %.4f" % (self.battery), 
#			(30, 30), 
#			self.fontFace, 
#			0.5, 
#			self.color, 
#			self.thickness, 
#			self.lineType, 
#			self.bottomLeftOrigin)
		
		self.cbQRMission()
		
#		# Status
#		cv2.putText(self.cv_image_clone, 
#			"Battery: %.4f" % (self.battery), 
#			(30, 30), 
#			self.fontFace, 
#			0.5, 
#			self.color, 
#			self.thickness, 
#			self.lineType, 
#			self.bottomLeftOrigin)
#			
#		cv2.putText(self.cv_image_clone, 
#			"QR Previous:%s : QR Current:%s" % (self.QR_Previous, self.QR_Current), 
#			(30, 60), 
#			self.fontFace, 
#			0.5, 
#			self.color, 
#			self.thickness, 
#			self.lineType, 
#			self.bottomLeftOrigin)
#			
#		cv2.putText(self.cv_image_clone, 
#			"QR1_Count: %d : QR2_Count: %d : QR3_Count: %d : QR4_Count: %d : QR5_Count: %d : QR6_Count: %d" % (self.QR1_Count,self.QR2_Count,self.QR3_Count,self.QR4_Count,self.QR5_Count,self.QR6_Count), 
#			(30, 90), 
#			self.fontFace, 
#			0.5, 
#			self.color, 
#			self.thickness, 
#			self.lineType, 
#			self.bottomLeftOrigin)
		
		cv2.imshow("Camera Preview -", self.cv_image_clone)
		cv2.waitKey(1)

	def cbQRMission(self):
		
		key = self.getKey()
		
		self.cbQRDetection()
		rospy.loginfo([self.missionCount, self.QR_Detected, self.QR_Previous, self.QR1_Count, self.cy, self.cv_image_clone.shape[0] // 2, self.takeoffed])
		
		if self.missionCount == 0 and key  == "v":
			rospy.loginfo("TakeOff!")
			self.bebopTakeoff_pub.publish(self.takeoff)
#			self.cbCamDown()
			self.twist.linear.x =  0.0
			self.twist.linear.y =  0.0
			self.twist.linear.z =  0.0

			self.twist.angular.x = 0.0
			self.twist.angular.y = 0.0
			self.twist.angular.z = 0.0
		
			self.bebopTwist_pub.publish(self.twist)
			
			self.isTakeoff()
#			self.cbCamDown()
			
		elif self.missionCount == 0 and self.QR_Detected == False and self.takeoffed == 1  and self.cy < self.cv_image_clone.shape[0] // 2:
			rospy.logerr("QR1 Not Detected!")
			self.twist.linear.x =  0.02
			self.twist.linear.y =  0.0
			self.twist.linear.z =  0.0

			self.twist.angular.x = 0.0
			self.twist.angular.y = 0.0
			self.twist.angular.z = 0.0

		elif self.missionCount == 0 and self.QR_Detected == True and self.takeoffed == 1 and self.cy > self.cv_image_clone.shape[0] // 2:
			rospy.logerr("QR1 Detected!")
#			if self.cx >= self.cv_image_clone.shape[1] // 2 + 10:
#				self.twist.linear.x =  0.0
#				self.twist.linear.y =  -0.01
#				self.twist.linear.z =  0.0

#				self.twist.angular.x = 0.0
#				self.twist.angular.y = 0.0
#				self.twist.angular.z = 0.0
#			
#			elif self.cx <= self.cv_image_clone.shape[1] // 2 - 10:
#				self.twist.linear.x =  0.0
#				self.twist.linear.y =  0.01
#				self.twist.linear.z =  0.0

#				self.twist.angular.x = 0.0
#				self.twist.angular.y = 0.0
#				self.twist.angular.z = 0.0
#				
#			else:
#				self.twist.linear.x =  0.0
#				self.twist.linear.y =  0.0
#				self.twist.linear.z =  0.0

#				self.twist.angular.x = 0.0
#				self.twist.angular.y = 0.0
#				self.twist.angular.z = 0.0
#				
#				self.missionCount += 1
				
			self.twist.linear.x =  0.0
			self.twist.linear.y =  0.0
			self.twist.linear.z =  0.0

			self.twist.angular.x = 0.0
			self.twist.angular.y = 0.0
			self.twist.angular.z = 0.0
			
			self.missionCount += 1
		
		elif self.missionCount == 1 and self.takeoffed == 1:
			rospy.loginfo("Rotate!")
			self.twist.linear.x =  0.0
			self.twist.linear.y =  0.0
			self.twist.linear.z =  0.0

			self.twist.angular.x = 0.0
			self.twist.angular.y = 0.0
			self.twist.angular.z = -0.05
			
			self.missionCount += 1
			
		elif self.missionCount == 2 and self.takeoffed == 1:
			rospy.loginfo("Land!")
			self.bebopLand_pub.publish(self.land)
			self.cbCamUp()
			self.twist.linear.x =  0.0
			self.twist.linear.y =  0.0
			self.twist.linear.z =  0.0

			self.twist.angular.x = 0.0
			self.twist.angular.y = 0.0
			self.twist.angular.z = 0.0
			
			self.missionCount += 1
			
			rospy.signal_shutdown("Mission Completed!")
			rospy.logerr("Mission Completed!")
			
			self.isLand()
			
		else:
			if (key == '\x03'):
				self.twist.linear.x =  0.0
				self.twist.linear.y =  0.0
				self.twist.linear.z =  0.0

				self.twist.angular.x = 0.0
				self.twist.angular.y = 0.0
				self.twist.angular.z = 0.0
				self.bebopLand_pub.publish(self.land)
				rospy.signal_shutdown("Mission Aborted!")
				rospy.logerr("Mission Aborted!")
			else:
				pass
				
		self.bebopTwist_pub.publish(self.twist)
			
	def cbQRDetection(self):
		# Converting to grayscale
		cv_image_gray = cv2.cvtColor(self.cv_image_clone, cv2.COLOR_BGR2GRAY)

		# QR Detection
		results = pyzbar.decode(cv_image_gray)
		
		if len(results) != 0:
			self.QR_Detected = True
		else:
			self.QR_Detected = False
		
		# loop over the detected barcodes
		for result in results:
		
			# extract the bounding box location of the barcode and draw the
			# bounding box surrounding the barcode on the image
			(x, y, w, h) = result.rect
			cv2.rectangle(self.cv_image_clone, (x, y), (x + w, y + h), (0, 0, 255), 2)
			
			self.cx, self.cy = x + w // 2, y + h // 2	# center
			cv2.line(self.cv_image_clone, (0, self.cy), (self.cv_image_clone.shape[1], self.cy), (0, 0, 255), 2)
			cv2.line(self.cv_image_clone, (self.cx, 0), (self.cx, self.cv_image_clone.shape[0]), (255, 0, 0), 2)
			cv2.circle(self.cv_image_clone, (self.cx, self.cy), 2, (0, 0, 255), 2)
		
			# the barcode data is a bytes object so if we want to draw it on
			# our output image we need to convert it to a string first
			barcodeData = result.data.decode("utf-8")
			self.QR_Current = barcodeData
#			barcodeType = result.type
		
#			# draw the barcode data and barcode type on the image
#			text = "{} ({})".format(barcodeData, barcodeType)
#			cv2.putText(self.cv_image_clone, text, (x, y - 10), self.fontFace,
#				self.fontScale, self.color, self.thickness)
			if self.QR_Previous != self.QR_Current:
		
				if self.QR_Current == self.QR1:
					self.QR1_Count += 1
				elif self.QR_Current == self.QR2:
					self.QR2_Count += 1
				elif self.QR_Current == self.QR3:
					self.QR3_Count += 1
				elif self.QR_Current == self.QR4:
					self.QR4_Count += 1
				elif self.QR_Current == self.QR5:
					self.QR5_Count += 1
				elif self.QR_Current == self.QR6:
					self.QR6_Count += 1
				
				self.QR_Previous = self.QR_Current
			else:
				self.QR_Previous = self.QR_Previous
				
	def getKey(self):
		if os.name == 'nt':
			return msvcrt.getch()

		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist:
			key = sys.stdin.read(1)
		else:
			key = ''

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key
		
	# Preview image + info
	def cbPreview(self):
		self.cbCamDown()
		if self.image_received:
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("Camera Preview Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)
		
	# Initialize
	rospy.init_node('camera_preview', anonymous=False)
	camera = CameraPreview()
	
#	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
#		r.sleep()
