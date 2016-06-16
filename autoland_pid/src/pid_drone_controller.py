#!/usr/bin/env python2

# A basic drone controller class for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials_getting_started

# This class implements basic control functionality which we will be using in future tutorials.
# It can command takeoff/landing/emergency as well as drone movement
# It also tracks the drone state based on navdata feedback

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('autoland_pid')
import rospy

# Import the messages we're interested in sending and receiving
from geometry_msgs.msg import Twist      # for sending commands to the drone
from std_msgs.msg import Empty           # for land/takeoff/emergency
from ardrone_autonomy.msg import Navdata # for receiving navdata feedback
from sensor_msgs.msg import CameraInfo

# Import image geometry to do 2d to 3d
import image_geometry
import sensor_msgs.msg

# Import PID control
from PID import PID

# An enumeration of Drone Statuses
from drone_status import DroneStatus


# Some Constants
COMMAND_PERIOD = 100 #ms


class BasicDroneController(object):
    def __init__(self):
        # Holds the current drone status
        self.status = -1
        self.targetSeen = 0

        #Define Tag
        self.tags = None
        
        # Set up image geometry
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_info = rospy.Subscriber('ardrone/front/camera_info', CameraInfo, self.callback)

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)
        
        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/ardrone/land',Empty, queue_size=1)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty, queue_size=1)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty, queue_size=1)
        
        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)
        self.command.angular.x = 1
        self.command.angular.y = 1
        self.SetCommand()
        
        #Autoland switch
        self.autoLand = False
        self.PIDenable = False

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

        # PID parameters
        #self.xyPID = (.0016, 0, 0.002)
        #PID parameters grabbed from tum
        self.xyPID = (.01, 0, .007)
        self.thetaPID = (0, 0, 0)
        self.zPID = (0, 0, 0)

        # PID control setup
        self.x = PID(P=self.xyPID[0], I=self.xyPID[1], D=self.xyPID[2], maxVal= .05)
        self.y = PID(P=self.xyPID[0], I=self.xyPID[1], D=self.xyPID[2], maxVal = .05)
        self.z = PID(P=self.zPID[0], I=self.zPID[1], D=self.zPID[2], maxVal = 1)
        self.z.setPoint = 1
        self.theta = PID(P=self.thetaPID[0], I=self.thetaPID[1], D=self.thetaPID[2])

###In order to do config on the fly, must implement ros package dynamic config.
    def SendPUp(self):
        self.xyPID = (self.xyPID[0] + .0001, self.xyPID[1], self.xyPID[2])
        self.x.setPID(self.xyPID)
        rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])

    def SendPDown(self):
        self.xyPID = (self.xyPID[0] - .0001, self.xyPID[1], self.xyPID[2])
        self.x.setPID(self.xyPID)
        rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])

    def SendIUp(self):
        self.xyPID = (self.xyPID[0], self.xyPID[1]+.0001, self.xyPID[2])
        self.x.setPID(self.xyPID)
        rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])

    def SendIDown(self):
        self.xyPID = (self.xyPID[0], self.xyPID[1]-.0001, self.xyPID[2])
        self.x.setPID(self.xyPID)
        rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])

    def SendDUp(self):
        self.xyPID = (self.xyPID[0], self.xyPID[1], self.xyPID[2]+.001)
        self.x.setPID(self.xyPID)
        rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])

    def SendDDown(self):
        self.xyPID = (self.xyPID[0], self.xyPID[1], self.xyPID[2]-.001)
        self.x.setPID(self.xyPID)
        rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])

    def callback(self, data):
        self.cam_model.fromCameraInfo(data)

    def ReceiveNavdata(self,navdata):
        # Extract information from navdata  
        self.status = navdata.state
        self.targetSeen = navdata.tags_count
        if self.targetSeen > 0:
            self.tags = [((navdata.tags_xc[i]), (navdata.tags_yc[i]), (navdata.tags_distance[i]), (navdata.tags_orientation[i])) for i in range(0,navdata.tags_count)]
        
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
        if self.tags is not None:
            uv = ((int(self.tags[0][0]*640.0/1000.0), int(self.tags[0][1]*360.0/1000.0)))
            return self.cam_model.projectPixelTo3dRay(uv)

    def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x  = pitch
        self.command.linear.y  = roll
        self.command.linear.z  = z_velocity
        self.command.angular.z = yaw_velocity
    
    def SetPIDCommand(self):
        # Convert to metric unit
        if self.tags is not None:
            vector = self.MakeCoordinates()
            dist   = self.tags[0][2]*.532
            # the values are switched for some reason
            x_dist = vector[1]*dist
            y_dist = vector[0]*dist
            angle  = self.tags[0][3]
            #rospy.loginfo("x_dist: %f, y_dist: %f, z_dist: %f, theta: %f \n", x_dist, y_dist, dist, angle)
        
            # Calculate error
            x_change = self.x.update(x_dist)
            y_change = self.y.update(y_dist)
            z_change = self.z.update(dist)
            t_change = self.theta.update(angle)
        
            # Update control
            rospy.loginfo("x: %f, y: %f, z: %f, t: %f \n" ,x_change, y_change, z_change, t_change)
            rospy.loginfo("p: %f, i: %f, d: %f", self.xyPID[0], self.xyPID[1], self.xyPID[2])
            if self.autoLand:
                self.SetCommand(y_change, x_change, t_change, z_change)

    def SendPIDEnable(self):
        # Start PID for AutoLand
        self.PIDenable = not self.PIDenable

    def SendAutoLand(self):
        # Start the auto landing sequence
        self.autoLand = not self.autoLand

    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        if self.PIDenable:
            self.SetPIDCommand()
        # elif self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
        self.pubCommand.publish(self.command)
