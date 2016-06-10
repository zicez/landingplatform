#!/usr/bin/env python

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('ardrone_tutorials')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist  	 # for sending commands to the drone
from std_msgs.msg import Empty       	 # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback

# Import image geometry to do 2d to 3d
import image_geometry
import sensor_msgs.msg

# Import PID control
import PID

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
	def __init__(self):
        # Holds the current drone status
        self.status = -1
        self.targetSeen = 0
        
        # Set up image geometry
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_info = rospy.Subscriber('ardrone/bottom/camera_info')
        self.cam_model.fromCameraInfo(self.cam_info)

		# Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
		self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata) 
		
		# Allow the controller to publish to the /ardrone/takeoff, land and reset topics
		self.pubLand    = rospy.Publisher('/ardrone/land',Empty)
		self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty)
		self.pubReset   = rospy.Publisher('/ardrone/reset',Empty)
		
		# Allow the controller to publish to the /cmd_vel topic and thus control the drone
		self.pubCommand = rospy.Publisher('/cmd_vel',Twist)

		# Setup regular publishing of control packets
		self.command = Twist()
		self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)
        
        #Autoland switch
        self.autoLand = 0

		# Land the drone if we are shutting down
		rospy.on_shutdown(self.SendLand)

        # PID control setup
        self.x = PID(.5, 0, 0)
        self.y = PID(.5, 0, 0)
        self.z = PID(.5, 0, 0)
        self.z.setPoint = .5
        self.theta = PID(.5, 0, 0)

	def ReceiveNavdata(self,navdata):
		# Extract information from navdata	
		self.status = navdata.state
        self.targetSeen = navdata.tags_count
        if self.targetSeen > 0:
            self.tags = [((navdata.tags_xc[i]), (navdata.tags_yc[i]), (navdata.tags_distance[i]), (navdata.tags_orientation[i]) for i in range(0,navdata.tags_count)]
        
	def SendTakeoff(self):
		# Send a takeoff message to the ardrone driver
		# Note we only send a takeoff message if the drone is landed - an unexpected takeoff is not good!
		if(self.status == DroneStatus.Landed):
			self.pubTakeoff.publish(Empty())

	def SendLand(self):
		# Send a landing message to the ardrone driver
		# Note we send this in all states, landing can do no harm
		self.pubLand.publish(Empty())

	def SendEmergency(self):
		# Send an emergency (or reset) message to the ardrone driver
		self.pubReset.publish(Empty())

    def MakeCoordinates(self):
        # Use image geometry to convert pixel to distance.
        uv = ((self.tags[0][0], self.tags[0][1]))
        return self.cam_model.projectPixelTo3dRay(uv)

	def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
		# Called by the main program to set the current command
		self.command.linear.x  = pitch
		self.command.linear.y  = roll
		self.command.linear.z  = z_velocity
		self.command.angular.z = yaw_velocity
    
    def SetPIDCommand(self):
        # Convert to metric unit
        vector = MakeCoordiantes()
        dist   = self.tags[0][2]
        x_dist = vector[0][0]*dist
        y_dist = vector[0][1]*dist
        angle  = self.tags[0][3]
        
        # Calculate error
        x_change = self.x.update(x_dist)
        y_change = self.y.update(y_dist)
        z_change = self.z.update(z)
        t_change = self.theta.update(angle)
        
        # Update control
        SetCommand(y_change, x_change, t_change, z_change)

    def SendAutoLand(self):
        # Start PID for AutoLand
        self.autoLand = !self.autoLand

	def SendCommand(self,event):
		# The previously set command is then sent out periodically if the drone is flying
		if self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering or self.autoLand = 0:
            self.pubCommand.publish(self.command)
        elif self.autoLand = 1:
            SetPIDCommand()
            self.pubCommand.publish(self.command)
