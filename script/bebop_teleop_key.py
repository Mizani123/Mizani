#!/usr/bin/env python
################################################################################
## {Description}: TeleOperation of Bebop Drone
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import numpy as np
import imutils
import random
import select
import os

# import the necessary ROS packages
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

import rospy

if os.name == 'nt':
	import msvcrt
else:
	import tty, termios

#Acceptable range for all fields are [-1..1]

MAX_LIN_VEL = 1
MAX_ANG_VEL = 1

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.05

msg = """
Control Your Bebop Drone!
---------------------------
Moving around:
		w					i
	a	s	d			j	k	l

w/s : increase/decrease linear (y) velocity
a/d : increase/decrease linear (x) velocity
i/k : increase/decrease linear (z) velocity
a/d : increase/decrease angular velocity

v : takeoff
b : land
space key : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
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

def vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel, target_angular_vel):
	return "currently:\tlinear xyz vel %s\t angular vel %s " % ([target_linear_x_vel, target_linear_y_vel, target_linear_z_vel],target_angular_vel)

def makeSimpleProfile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output

def constrain(input, low, high):
	if input < low:
		input = low
	elif input > high:
		input = high
	else:
		input = input

	return input

def checkLinearLimitVelocity(vel):
	vel = constrain(vel, -MAX_LIN_VEL, MAX_LIN_VEL)
	return vel

def checkAngularLimitVelocity(vel):
	vel = constrain(vel, -MAX_ANG_VEL, MAX_ANG_VEL)
	return vel

if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('bebop_teleop', anonymous=False)
	pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
	pubTakeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
	pubLand = rospy.Publisher('/bebop/land', Empty, queue_size=10)
	
	bebopCamdown_topic = "/bebop/camera_control"
	bebopCamdown_pub = rospy.Publisher(
						bebopCamdown_topic, 
						Twist, 
						queue_size=1
						)

	status = 0
	target_linear_x_vel   = 0.0
	target_linear_y_vel   = 0.0
	target_linear_z_vel   = 0.0
	
	control_linear_x_vel  = 0.0
	control_linear_y_vel  = 0.0
	control_linear_z_vel  = 0.0
	
	target_angular_vel  = 0.0
	control_angular_vel = 0.0

	try:
		print(msg)
		while True:
			key = getKey()
			# translate left
			if key == 'a' :
				target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel + LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# translate right
			elif key == 'd' :
				target_linear_y_vel = checkLinearLimitVelocity(target_linear_y_vel - LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# translate backward
			elif key == 's' :
				target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel - LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# translate forward
			elif key == 'w' :
				target_linear_x_vel = checkLinearLimitVelocity(target_linear_x_vel + LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# ascend
			elif key == 'i' :
				target_linear_z_vel = checkLinearLimitVelocity(target_linear_z_vel + LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# decend
			elif key == 'k' :
				target_linear_z_vel = checkLinearLimitVelocity(target_linear_z_vel - LIN_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# rotate clockwise
			elif key == 'l' :
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# rotate counter clockwise
			elif key == 'j' :
				target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# take off
			elif key == 'v' :
				takeoff = Empty()
				pubTakeoff.publish(takeoff)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# land
			elif key == 'b' :
				land = Empty()
				pubLand.publish(land)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# land
			elif key == 'f' :
				cam = Twist()
				cam.angular.y = 90.0
				bebopCamdown_pub.publish(cam)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			# land
			elif key == 'g' :
				cam = Twist()
				cam.angular.y = -90.0
				bebopCamdown_pub.publish(cam)
				status = status + 1
				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
				
			elif key == ' ':
				target_linear_x_vel   = 0.0
				target_linear_y_vel   = 0.0
				target_linear_z_vel   = 0.0
				
				control_linear_x_vel  = 0.0
				control_linear_y_vel  = 0.0
				control_linear_z_vel  = 0.0
				
				target_angular_vel  = 0.0
				control_angular_vel = 0.0

				print(vels(target_linear_x_vel, target_linear_y_vel, target_linear_z_vel,target_angular_vel))
			else:
				if (key == '\x03'):
					break

			if status == 2 :
				print(msg)
				status = 0

			twist = Twist()

			control_linear_x_vel = makeSimpleProfile(control_linear_x_vel, target_linear_x_vel, (LIN_VEL_STEP_SIZE/2.0))
			control_linear_y_vel = makeSimpleProfile(control_linear_y_vel, target_linear_y_vel, (LIN_VEL_STEP_SIZE/2.0))
			control_linear_z_vel = makeSimpleProfile(control_linear_z_vel, target_linear_z_vel, (LIN_VEL_STEP_SIZE/2.0))
			twist.linear.x = control_linear_x_vel
			twist.linear.y = control_linear_y_vel
			twist.linear.z = control_linear_z_vel

			control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
			twist.angular.x = 0.0
			twist.angular.y = 0.0
			twist.angular.z = control_angular_vel

			pub.publish(twist)

	except:
		print(e)

	finally:
		twist = Twist()
		twist.linear.x = 0.0
		twist.linear.y = 0.0
		twist.linear.z = 0.0
		
		twist.angular.x = 0.0
		twist.angular.y = 0.0
		twist.angular.z = 0.0
		
		pub.publish(twist)

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
