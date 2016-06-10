#!/usr/bin/env python

# This version sends command to tum ardrone for control. 

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies

#This file is based on the tutorial mentioned above.
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty, String       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata# for receiving navdata feedback

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms 

class BasicDroneController(object):
	def __init__(self):
		# Holds the current drone status
		self.status = -1
		self.targetSeen = 0
		self.tags = []
		self.coordinates = []
		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		# Send commands to tum_ardrone
		self.pubAutoLand = rospy.Publisher('/tum_ardrone/com', String)

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

	def ReceiveNavdata(self,navdata):
		# Extract information from navdata
		self.status = navdata.state
		self.targetSeen = navdata.tags_count
		if self.targetSeen > 0:
			self.tags = [((navdata.tags_xc[i]),(navdata.tags_yc[i]),(navdata.tags_distance[i]), (navdata.tags_orientation[i]) ) for i in range(0,navdata.tags_count)]

	def MakeCoordinates(self):
		#Transform the navdata into landing targets
		self.coordinates = [((500-self.tags[0][0])/500, (500-self.tags[0][1])/100, -.1,  0)]

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())


	def SendAutoLand(self):
		# Send autoland
		if self.tags != []:
			self.MakeCoordinates()
			self.pubAutoLand.publish("c clearCommands")
			self.pubAutoLand.publish("c setReference $POSE$")
			self.pubAutoLand.publish("c goto " + str(' '.join(map(str,self.coordinates[0]))))
