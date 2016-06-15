#!/usr/bin/env python
import roslib; roslib.load_manifest('quadcopter_control')
import rospy
import cv
import math
from std_msgs.msg import String
from std_msgs.msg import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from vision_location.msg import QuadcopterLocation
from time import time
from vision_location.msg import FieldBoundary


import sys, select, termios, tty, math
from numpy  import *
import random

import threading
from collections import deque


# Initialize current and target location

current_yaw = 0
current_pitch = 0
current_roll = 0
altitude = 0
battery = 0

# Initialize running variables

kp = 0.3
ki = 0
kd = 6.0
dt = 0.033333

previous_ex = 0
previous_ey = 0
previous_ez = 0

current_ex = 0
current_ey = 0
current_ez = 0

sum_ex = 0
sum_ey = 0
sum_ez = 0		

start_count = 0
lost_count = 0


# Initialize Yaw

YawInitializeCount = 0
InitialYaw = 0



def handleLocation(msg):

	global current
	global lock

	if (msg.no_detection < 1 and msg.has_pcd_loc > 0):

		lock.acquire()

		current = msg.integrated_loc

		lock.release()


def handleNavData(nav_data):

	global current_yaw
	global current_pitch
	global current_roll
	global altitude
	global battery
	global velocity
	global YawInitializeCount
	global InitialYaw

	current_yaw = nav_data.attitude.x 
	current_pitch = nav_data.attitude.y 
	current_roll = nav_data.attitude.z 
	altitude = nav_data.altitude
	battery = nav_data.battery
	velocity = nav_data.velocity

	if (YawInitializeCount == 0):
		InitialYaw = current_yaw

	while (YawInitializeCount < 100):
		YawInitializeCount = YawInitializeCount + 1
		InitialYaw = (InitialYaw * (YawInitializeCount-1) + current_yaw)/YawInitializeCount

	InitialAttitude = Twist()
	InitialAttitude.angular.x = InitialYaw

	pub_initial_attitude.publish(InitialAttitude)



def handleKeys(msg):

	global kp
	global ki
	global kd

	kp = kp + msg.linear.x;
	ki = ki + msg.linear.y;
	kd = kd + msg.linear.z;
	print 'Kp: ',kp,' Ki: ',ki,' Kd: ',kd,'\r'


def handleTargets(msg):

	global current
	global velocity
	global target

	global start_count

	global InitialYaw


	InitialYaw = InitialYaw - msg.angular.z * 2.5

	if InitialYaw > 180:
		InitialYaw = InitialYaw - 360

	if InitialYaw < -180:
		InitialYaw = InitialYaw + 360

	InitialAttitude = Twist()
	InitialAttitude.angular.x = InitialYaw

	pub_initial_attitude.publish(InitialAttitude)

	target.point.x = target.point.x + msg.linear.x
	target.point.y = target.point.y + msg.linear.y
	target.point.z = target.point.z + msg.linear.z
	
	output = QuadcopterLocation()

	output.integrated_loc = current
	output.target = target
	output.velocity = velocity

	if (start_count > 1):
		output.locked = 1
	else:
		output.locked = 0

	output.attitude.x = current_yaw 
	output.attitude.y = current_pitch 
	output.attitude.z = current_roll 
	output.altitude = altitude
	output.battery = battery

	pub_loc.publish(output)


def quadcopter_control(data):

	global pub
	global time
	global dt

	global start_count
	global lost_count
	
	global current
	global previous
	global target
	global velocity

	global current_ex
	global current_ey
	global current_ez

	global previous_ex
	global previous_ey
	global previous_ez

	global sum_ex
	global sum_ey
	global sum_ez

	global current_yaw
	global InitialYaw


	output = QuadcopterLocation()

	action = Twist()
	action.linear.x = 0
	action.linear.y = 0
	action.linear.z = 0
	action.angular.x = 0
	action.angular.y = 0
	action.angular.z = 0

	dt = rospy.get_time() - time
    	time = rospy.get_time()

	# print "Current: ",cx," ",cy," vs Target: ",tx," ",ty," \r"

	output.integrated_loc = current
	output.target = target
	output.velocity = velocity

	output.locked = 0

	output.attitude.x = current_yaw 
	output.attitude.y = current_pitch 
	output.attitude.z = current_roll 
	output.altitude = altitude
	output.battery = battery



	if ((math.fabs(current.point.x - target.point.x) < 0.15)  & (math.fabs(current.point.y - target.point.y) < 0.15)):
		start_count = start_count + 1
		lost_count = 0

	
	if (start_count > 1):

		lost_count = 0
		output.locked = 1
		current_ex = target.point.y - previous.point.y
		current_ey = target.point.x - previous.point.x
		current_ez = target.point.z - previous.point.z

		sum_ex = sum_ex + current_ex*dt
		sum_ey = sum_ey + current_ey*dt
		sum_ez = sum_ez + current_ez*dt
	
		# print 'ex = ', -1 *newex, ' ey = ', -1 *newey, ' SumEx = ', sumey, ' SumEy = ', sumex 

		action.linear.x = -1 * (kp * current_ex + ki * sum_ex + kd * (current_ex-previous_ex)) # / (rospy.get_time() - time))
		action.linear.y = -1 * (kp * current_ey + ki * sum_ey + kd * (current_ey-previous_ey)) # / (rospy.get_time() - time))
		action.linear.z = 0 #-1 * (kp * current_ez + ki * sum_ez + kd * (current_ez-previous_ez)) # / (rospy.get_time() - time))

		if (current_yaw - InitialYaw > -180 and current_yaw - InitialYaw < 180):

			if (current_yaw < InitialYaw):
				action.angular.z = -1 * (InitialYaw - current_yaw) * 0.01

			if (current_yaw > InitialYaw):
				action.angular.z = -1 * (InitialYaw - current_yaw) * 0.01

		if (current_yaw - InitialYaw <= -180 or current_yaw - InitialYaw >= 180):

			if (InitialYaw < 0):
				current_yaw = current_yaw - 360

			if (InitialYaw > 0):
				current_yaw = current_yaw + 360

			if (current_yaw < InitialYaw):
				action.angular.z = -1 * (InitialYaw - current_yaw) * 0.01

			if (current_yaw > InitialYaw):
				action.angular.z = -1 * (InitialYaw - current_yaw) * 0.01


		#action.angular.x = 0.07

		#print 'Turning: ',action.angular.z,'\r'

		output.action.point.x = action.linear.x
		output.action.point.y = action.linear.y
		output.action.point.z = action.linear.z
		

		previous.point.x = current.point.x
		previous.point.y = current.point.y
		previous.point.z = current.point.z
		previous_ex = current_ex
		previous_ey = current_ey
		previous_ez = current_ez
	
		pub.publish(action)

	pub_loc.publish(output)


def handleLock(data):

	global start_count

	start_count = 2



def handleUnlock(data):

	global start_count

	global current_ex
	global current_ey
	global current_ez

	global previous_ex
	global previous_ey
	global previous_ez

	global sum_ex
	global sum_ey
	global sum_ez

	start_count = 0
	sum_ex = 0
	sum_ey = 0
	sum_ez = 0
	previous_ex = 0
	previous_ey = 0
	previous_ez = 0

	current_ex = 0
	current_ey = 0
	current_ez = 0



if __name__ == "__main__":
	
	global pub
	global time
	global lock
	global current
	global previous
	global target
	global velocity

	lock = threading.Lock()
	
	rospy.init_node('quadcopter_pid_control', anonymous=True)

        time = rospy.get_time()
	current = PointStamped()
	previous = PointStamped()
	velocity = PointStamped()
	target = PointStamped()

	current.header.frame_id = "/world"
	previous.header.frame_id = "/world"
	velocity.header.frame_id = "/world"
	target.header.frame_id = "/world"

	pub = rospy.Publisher('/cmd_vel', Twist)
	pub_loc = rospy.Publisher('/quadcopter_loc', QuadcopterLocation)

	pub_initial_attitude = rospy.Publisher('/quadcopter_initial_attitude', Twist)

	rospy.Subscriber("/integrated_loc", QuadcopterLocation, handleLocation)

	rospy.Subscriber("/cmd_target", Twist, handleTargets)
	rospy.Subscriber("/psulu_target", Twist, handlePSuluTargets)

	rospy.Subscriber("/quadcopter_nav_data", QuadcopterLocation, handleNavData)

	rospy.Subscriber("/ardrone/lock",Empty, handleLock)
	rospy.Subscriber("/ardrone/unlock",Empty, handleUnlock)

	rospy.Subscriber("/pid_timer",Empty, quadcopter_control)



	rospy.spin()
