#!/usr/bin/env python
import roslib; roslib.load_manifest('autoland_pid')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty

from pid_drone_controller import BasicDroneController

# define the default mapping between joystick buttons and their corresponding actions
ButtonEmergency = 9
ButtonLand      = 7
ButtonTakeoff   = 5

# define the default mapping between joystick axes and their corresponding directions
AxisRoll        = 0
AxisPitch       = 1
AxisYaw         = 2
AxisZ           = 3

# define the default scaling to apply to the axis inputs. useful where an axis is inverted
ScaleRoll       = 0.3
ScalePitch      = 0.3
ScaleYaw        = 0.4
ScaleZ = 0.4

def ReceiveJoystickMessage(data):
	if data.buttons[ButtonEmergency]==1:
		rospy.loginfo("Emergency Button Pressed")
		controller.SendEmergency()
	elif data.buttons[ButtonLand]==1:
		rospy.loginfo("Land Button Pressed")
		controller.SendLand()
	elif data.buttons[ButtonTakeoff]==1:
		rospy.loginfo("Takeoff Button Pressed")
		controller.SendTakeoff()
	else:
		controller.SetCommand(data.axes[AxisRoll]/ScaleRoll,data.axes[AxisPitch]/ScalePitch,data.axes[AxisYaw]/ScaleYaw,data.axes[AxisZ]/ScaleZ)

if __name__=="__main__":

	rospy.init_node('joystick_control')

	controller = BasicDroneController()
	rospy.Subscriber("/joy", Joy, ReceiveJoystickMessage)


	rospy.spin()




