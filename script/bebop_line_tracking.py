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
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from bebop_msgs.msg import CommonCommonStateBatteryStateChanged  # For battery percentage

import rospy

class CameraLinePreview:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False
		self.Kp = 0.112	# Ku=0.14 T=6. PID: p=0.084,i=0.028,d=0.063. PD: p=0.112, d=0.084/1. P: p=0.07
		self.Ki = 0
		self.kd = 1
		self.integral = 0
		self.derivative = 0
		self.last_error = 0
		self.Kp_ang = 0.01	# Ku=0.04 T=2. PID: p=0.024,i=0.024,d=0.006. PD: p=0.032, d=0.008. P: p=0.02/0.01
		self.Ki_ang = 0
		self.kd_ang = 0
		self.integral_ang = 0
		self.derivative_ang = 0
		self.last_ang = 0
		self.was_line = 0
		self.line_side = 0
		self.battery = 0
		self.line_back = 1
		self.landed = 0
		self.takeoffed = 0
		self.error = []
		self.angle = []
		self.fly_time = 0.0
		self.start = 0.0
		self.stop = 0.0
		self.velocity = 0.06

		rospy.logwarn("Camera (Line) Preview Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.bebopImage_topic = "/bebop/image_raw/compressed"
		self.bebopImage_sub = rospy.Subscriber(
						self.bebopImage_topic, 
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
		
		cv2.imshow("Camera (Line) Preview", self.cv_image_clone)
		cv2.waitKey(1)

	# Zoom-in the image
	def cbZoom(self, scale=20):
		self.cv_image_clone = self.cv_image.copy()
		self.height, self.width, _ = self.cv_image_clone.shape
		
		# prepare the crop
		self.centerX, self.centerY = int(self.height / 2), int(self.width / 2)
		self.radiusX, self.radiusY = int(scale * self.height / 100), int(scale * self.width / 100)

		self.minX, self.maxX = self.centerX - self.radiusX, self.centerX + self.radiusX
		self.minY, self.maxY = self.centerY - self.radiusY, self.centerY + self.radiusY

		self.cv_image_clone = self.cv_image_clone[self.minX:self.maxX, self.minY:self.maxY]
		self.cv_image_clone = cv2.resize(self.cv_image_clone, (self.width, self.height))
		self.cv_image_clone = cv2.add(self.cv_image_clone, np.array([-50.0]))

	# Detect the line
	def cbLineDetection(self):
		self.cbZoom()
		
		# Create a mask
		self.cv_image_hsv = cv2.cvtColor(self.cv_image_clone, cv2.COLOR_BGR2HSV)
#		self.mask = cv2.inRange(self.cv_image_clone, (130, 130, 130), (255, 255, 255))
		self.mask = cv2.inRange(self.cv_image_hsv, (0, 0, 0), (179, 50, 255))
#		self.kernel = np.ones((3, 3), np.uint8)
#		self.mask = cv2.erode(self.mask, self.kernel, iterations=5)
#		self.mask = cv2.dilate(self.mask, self.kernel, iterations=9)
		self.mask = cv2.bitwise_and(self.cv_image_clone, self.cv_image_clone, mask=self.mask)
		self.mask2 = cv2.bitwise_not(self.mask)
#		_, contours_blk, _ = cv2.findContours(self.mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#		contours_blk.sort(key=cv2.minAreaRect)
#		
#		if len(contours_blk) > 0 and cv2.contourArea(contours_blk[0]) > 5000:
#			self.was_line = 1
#			blackbox = cv2.minAreaRect(contours_blk[0])
#			(x_min, y_min), (w_min, h_min), angle = blackbox
#			
#			if angle < -45:
#				angle = 90 + angle
#			if w_min < h_min and angle > 0:
#				angle = (90 - angle) * -1
#			if w_min > h_min and angle < 0:
#				angle = 90 + angle
#				
#			setpoint = self.cv_image_clone.shape[1] / 2
#			error = int(x_min - setpoint)
#			self.error.append(error)
#			self.angle.append(angle)
#			normal_error = float(error) / setpoint

#			if error > 0:
#				self.line_side = 1  # line in right
#			elif error <= 0:
#				self.line_side = -1  # line in left

#			self.integral = float(self.integral + normal_error)
#			self.derivative = normal_error - self.last_error
#			self.last_error = normal_error


#			error_corr = -1 * (self.Kp * normal_error + self.Ki * self.integral + self.kd * self.derivative)  # PID controler
#			# print("error_corr:  ", error_corr, "\nP", normal_error * self.Kp, "\nI", self.integral* self.Ki, "\nD", self.kd * self.derivative)

#			angle = int(angle)
#			normal_ang = float(angle) / 90

#			self.integral_ang = float(self.integral_ang + angle)
#			self.derivative_ang = angle - self.last_ang
#			self.last_ang = angle

#			ang_corr = -1 * (self.Kp_ang * angle + self.Ki_ang * self.integral_ang + self.kd_ang * self.derivative_ang)  # PID controler

#			box = cv2.boxPoints(blackbox)
#			box = np.int0(box)

#			cv2.drawContours(self.cv_image_clone, [box], 0, (0, 0, 255), 3)

#			cv2.putText(self.cv_image_clone, "Angle: " + str(angle), (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2,
#			cv2.LINE_AA)

#			cv2.putText(self.cv_image_clone, "Error: " + str(error), (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2,
#			cv2.LINE_AA)
#			cv2.line(self.cv_image_clone, (int(x_min), 200), (int(x_min), 250), (255, 0, 0), 3)
#		
#		cv2.putText(self.cv_image_clone, "battery: " + str(self.battery) + "%", (570, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
#		cv2.imshow("Camera (Line) Preview", self.cv_image_clone)
		cv2.imshow("Camera (Line) Preview1", self.mask)
		cv2.imshow("Camera (Line) Preview2", self.mask2)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
			self.cbLineDetection()
		else:
			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("Camera (Line) Preview Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_line_preview', anonymous=False)
	camera = CameraLinePreview()
	camera.cam_down()
	
#	r = rospy.Rate(10)
	
	# Camera (Line) Preview
	while not rospy.is_shutdown():
		camera.cbPreview()
#		r.sleep()
