#!/usr/bin/env python2

# The Keyboard Controller Node for the tutorial "Up and flying with the AR.Drone and ROS | Getting Started"
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('autoland_pid')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from pid_drone_controller4 import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui


# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
    PitchForward     = QtCore.Qt.Key.Key_W
    PitchBackward    = QtCore.Qt.Key.Key_S
    RollLeft         = QtCore.Qt.Key.Key_A
    RollRight        = QtCore.Qt.Key.Key_D
    YawLeft          = QtCore.Qt.Key.Key_Q
    YawRight         = QtCore.Qt.Key.Key_E
    IncreaseAltitude = QtCore.Qt.Key.Key_R
    DecreaseAltitude  = QtCore.Qt.Key.Key_F
    Takeoff          = QtCore.Qt.Key.Key_Y
    Land             = QtCore.Qt.Key.Key_H
    Emergency        = QtCore.Qt.Key.Key_Space
    AutoLand         = QtCore.Qt.Key.Key_X
    PUp              = QtCore.Qt.Key.Key_1
    PDown            = QtCore.Qt.Key.Key_2
    IUp              = QtCore.Qt.Key.Key_3
    IDown            = QtCore.Qt.Key.Key_4
    DUp              = QtCore.Qt.Key.Key_5
    DDown            = QtCore.Qt.Key.Key_6


# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
    def __init__(self):
        super(KeyboardController,self).__init__()
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0
        self.z_velocity = 0

        self.pitch_delta = 0.15
        self.roll_delta = 0.15
        self.yaw_delta = 0.2
        self.z_delta = 0.2

    # We add a keyboard handler to the DroneVideoDisplay to react to keypresses
    def keyPressEvent(self, event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if controller is not None and not event.isAutoRepeat():
            # Handle the important cases first!
            if key == KeyMapping.Emergency:
                controller.SendEmergency()
            elif key == KeyMapping.Takeoff:
                controller.SendTakeoff()
            elif key == KeyMapping.Land:
                controller.SendLand()
            elif key == KeyMapping.AutoLand:
                controller.SendAutoLand()
            elif key == KeyMapping.PUp:
                controller.SendPUp()
            elif key == KeyMapping.PDown:
                controller.SendPDown()
            elif key == KeyMapping.IUp:
                controller.SendIUp()
            elif key == KeyMapping.IDown:
                controller.SendIDown()
            elif key == KeyMapping.DUp:
                controller.SendDUp()
            elif key == KeyMapping.DDown:
                controller.SendDDown()
            else:
                # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
                if key == KeyMapping.YawLeft:
                    self.yaw_velocity += self.yaw_delta
                elif key == KeyMapping.YawRight:
                    self.yaw_velocity += -self.yaw_delta

                elif key == KeyMapping.PitchForward:
                    self.pitch += self.pitch_delta
                elif key == KeyMapping.PitchBackward:
                    self.pitch += -self.pitch_delta

                elif key == KeyMapping.RollLeft:
                    self.roll += self.roll_delta
                elif key == KeyMapping.RollRight:
                    self.roll += -self.roll_delta

                elif key == KeyMapping.IncreaseAltitude:
                    self.z_velocity += self.z_delta
                elif key == KeyMapping.DecreaseAltitude:
                    self.z_velocity += -self.z_delta

            # finally we set the command to be sent. The controller handles sending this at regular intervals
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


    def keyReleaseEvent(self,event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if controller is not None and not event.isAutoRepeat():
            # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
            # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
            if key == KeyMapping.YawLeft:
                self.yaw_velocity -= self.yaw_delta
            elif key == KeyMapping.YawRight:
                self.yaw_velocity -= -self.yaw_delta

            elif key == KeyMapping.PitchForward:
                self.pitch -= self.pitch_delta
            elif key == KeyMapping.PitchBackward:
                self.pitch -= -self.pitch_delta

            elif key == KeyMapping.RollLeft:
                self.roll -= self.roll_delta
            elif key == KeyMapping.RollRight:
                self.roll -= -self.roll_delta

            elif key == KeyMapping.IncreaseAltitude:
                self.z_velocity -= self.z_delta
            elif key == KeyMapping.DecreaseAltitude:
                self.z_velocity -= -self.z_delta

            # finally we set the command to be sent. The controller handles sending this at regular intervals
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)



# Setup the application
if __name__=='__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ardrone_keyboard_controller', log_level=rospy.DEBUG)

    # Now we construct our Qt Application and associated controllers and windows
    app = QtGui.QApplication(sys.argv)
    controller = BasicDroneController()
    display = KeyboardController()

    display.show()

    # executes the QT application
    status = app.exec_()

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
