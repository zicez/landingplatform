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
from ar_track_alvar_msgs.msg import AlvarMarkers #for tracking targets

# Import PID control
from PID import PID
from PD import PD2

# An enumeration of Drone Statuses
from drone_status import DroneStatus

# Import tf library to do coordinate transformation
import tf

# Some Constants
COMMAND_PERIOD = 50 #ms


class BasicDroneController(object):
    def __init__(self):
        # Holds the current drone status
        self.status = -1
        self.targetSeen = 0

        #Define Tag
        self.tags = None

        # Subscribe to the /ardrone/navdata topic, of message type navdata, and call self.ReceiveNavdata when a message is received
        self.subNavdata = rospy.Subscriber('/ardrone/navdata',Navdata,self.ReceiveNavdata)

        # Track alvar markers
        self.subAlvarData = rospy.Subscriber('/ar_pose_marker', AlvarMarkers , self.ReceiveAlvar)

        # Set up tf listener
        self.listener = tf.TransformListener()

        # Allow the controller to publish to the /ardrone/takeoff, land and reset topics
        self.pubLand    = rospy.Publisher('/ardrone/land',Empty, queue_size=1)
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff',Empty, queue_size=1)
        self.pubReset   = rospy.Publisher('/ardrone/reset',Empty, queue_size=1)

        # Allow the controller to publish to the /cmd_vel topic and thus control the drone
        self.pubCommand = rospy.Publisher('/cmd_vel',Twist, queue_size=1)

        # Allow the controller to publish to the /toggle topic for engaging the landing platform
        self.pubPlatform = rospy.Publisher('/toggle', Empty, queue_size=1)

        # Setup regular publishing of control packets
        self.command = Twist()
        self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)

        #Initializing values
        self.command.angular.x = 1
        self.command.angular.y = 1
        self.SetCommand()

        #Autoland switch
        self.autoLand = False #hovering over the large target
        self.autoLandSmall = False #land on small target
        self.PIDenable = False #activate this first, turn on controller.

        # Land the drone if we are shutting down
        rospy.on_shutdown(self.SendLand)

        # PID parameters
        #previous values that worked xyPID = (.125, 0, 3) when the only axis working
        #self.xyPID = (.15, 0, 8) #d 1.5
        #self.thetaPID = (0.05, 0, 0)

        # PID control setup
        self.zPID = (-0.3, -0.001, -0.6)
        self.z = PID(P=self.zPID[0], I=self.zPID[1], D=self.zPID[2], maxVal = .2)
        self.z.setPoint(-.8)
        self.lastSeenPos = ()
        self.lastSeenOri = ()

        # PD theta setup
        self.theta = PID(P = -0.06, I = 0.00001, D = .0005, maxVal = .1)
        self.theta.setPoint(0)

        #PD control setup
        #.46,4.822,.6 D too low
        #.46,4.832,.35 D too high
        # .46,4.827,.35 D too high DD too low
        # self.dx = PD2(.46, 4.824, .45) P is too high (too many micro adjustment near the target)
        # self.dx = PD2(.46, 4.824, .45)
        # self.dy = PD2(.46, 4.824, .45)
        self.dx = PD2(.46, 4.824, .45)
        self.dy = PD2(.46, 4.824, .45)

        #initializing speed variable
        self.speed = (0,0,0)

        self.last_time = None

    def ReceiveAlvar(self, data):
        br = tf.TransformBroadcaster()
        self.tags = data.markers
        if self.tags != []:
            for i in self.tags:
                if i.id == 0:
                    self.lastSeenPos = i.pose.pose.position
                    self.lastSeenOri = i.pose.pose.orientation
                    pos = i.pose.pose.position
                    ori = i.pose.pose.orientation
                    br.sendTransform((pos.x, pos.y, pos.z), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), "target", "ardrone_base_bottomcam")
        elif self.lastSeenPos == ():
            pass
        elif self.PIDenable:
            br.sendTransform((self.lastSeenPos.x, self.lastSeenPos.y, self.lastSeenPos.z), (self.lastSeenOri.x, self.lastSeenOri.y, self.lastSeenOri.z, self.lastSeenOri.w), rospy.Time.now(), "target", "ardrone_base_bottomcam")
            rospy.loginfo("Target lost: using previous location data point")

    def ReceiveNavdata(self,navdata):
        # Extract information from navdata
        self.status = navdata.state
        self.speed = (navdata.vx*.0001, navdata.vy*.0001, navdata.vz*.0001)

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

    def SendPlatform(self):
        #Send an empty message to the platform to engage opening or closing
        self.pubPlatform.publish(Empty())

    def SetCommand(self,roll=0,pitch=0,yaw_velocity=0,z_velocity=0):
        # Called by the main program to set the current command
        self.command.linear.x  = pitch
        self.command.linear.y  = roll
        self.command.linear.z  = z_velocity
        self.command.angular.z = yaw_velocity

    def SetPIDCommand(self):
        # Does frame transformation from POV of the camera into the drone's roll, pitch, yaw orientation.
        if self.listener.frameExists("/target"):
            try:
                (vector, rot) = self.listener.lookupTransform('/ardrone_base_link', "/target", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
            else:
                if self.last_time is None:
                    self.last_time = rospy.Time.now()
                    dt = 0.0
                else:
                    time = rospy.Time.now()
                    dt = (time - self.last_time).to_sec()
                    self.last_time = time
                quaternion = (rot[0], rot[1], rot[2], rot[3])
                euler = tf.transformations.euler_from_quaternion(quaternion)
                x_change = self.dx.update(vector[0], self.speed[0], dt)
                y_change = self.dy.update(vector[1], self.speed[1], dt)
                z_change = self.z.update(vector[2])
                t_change = self.theta.update(euler[2])
                # Update control
                rospy.loginfo("x_loc: %f, y_loc: %f, z_loc: %f, t_loc: %f \n", vector[0], vector[1], vector[2], euler[2])
                rospy.loginfo("x_speed: %f, y_speed: %f, z_speed: %f\n", self.speed[0], self.speed[1], self.speed[2])
                rospy.loginfo("x: %f, y: %f, z: %f, t: %f \n", x_change, y_change, z_change, t_change)
                if self.autoLand:
                    self.SetCommand(y_change, x_change, t_change, z_change)
        else:
            rospy.loginfo("Target not yet initialized")


    def SendPIDEnable(self):
        # Start PID for AutoLand
        self.PIDenable = not self.PIDenable

    def SendAutoLand(self):
        # Start the auto landing sequence
        self.autoLand = not self.autoLand

    def SendAutoLandSmall(self):
        # Start the auto landing sequence on the small target
        self.autoLandSmall = not self.autoLandSmall
        if self.z.getPoint() == -.8:
            self.z.setPoint(-.4)
        else:
            self.z.setPoint(-.8)

    def SendCommand(self,event):
        # The previously set command is then sent out periodically if the drone is flying
        if self.PIDenable:
            self.SetPIDCommand()
        # elif self.status == DroneStatus.Flying or self.status == DroneStatus.GotoHover or self.status == DroneStatus.Hovering:
        self.pubCommand.publish(self.command)
