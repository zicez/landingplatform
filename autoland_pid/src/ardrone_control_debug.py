#!/usr/bin/env python
import roslib; roslib.load_manifest('autoland_pid')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard and Publishing to Twist!
---------------------------
up/down:	move forward/backward
left/right:	move left/right
w/s:		increase/decrease altitude
a/d:		turn left/right
t/l:		takeoff/land
r:		reset (toggle emergency state)
g/j:		move target left/right
y/h:		move target up/down
b/m:		turn target orientation left/right
o/p:		lock/unlock target
z/x:		increase/decrease kp
c/v:		increase/decrease ki	
b/n:		increase/decrease kd		


please don't have caps lock on.
CTRL+c to quit
"""
move_bindings = {
	68:('linear', 'y', 0.15), #left
	67:('linear', 'y', -0.15), #right
	65:('linear', 'x', 0.15), #forward
	66:('linear', 'x', -0.15), #back
	'w':('linear', 'z', 0.3),
	's':('linear', 'z', -0.2),
	'a':('angular', 'z', 0.4),
	'd':('angular', 'z', -0.4),
}

target_bindings = {
	'g':('linear', 'x', 0.02), #up
	'j':('linear', 'x', -0.02), #down
	'y':('linear', 'y', -0.02), #left
	'h':('linear', 'y', 0.02), #right
	'b':('angular', 'z', 0.4),
	'm':('angular', 'z', -0.4),
}

pid_bindings = {
	'z':('linear', 'x', 0.01), #kp+
	'x':('linear', 'x', -0.01), #kp-
	'c':('linear', 'y', 0.0001), #ki+
	'v':('linear', 'y', -0.0001), #ki-
	'b':('linear', 'z', 0.01), #kd+
	'n':('linear', 'z', -0.01), #kd-
}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	print msg

	move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
	target_pub = rospy.Publisher('cmd_target', Twist, queue_size=1)
	pid_pub = rospy.Publisher('cmd_key', Twist, queue_size=1)
	land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
	reset_pub = rospy.Publisher('/ardrone/reset', Empty, queue_size=1)
	takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
	nav_request_pub = rospy.Publisher('/ardrone/nav_request', Empty, queue_size=1)
	lock_pub = rospy.Publisher('/ardrone/lock', Empty, queue_size=1)
	unlock_pub = rospy.Publisher('/ardrone/unlock', Empty, queue_size=1)

	rospy.init_node('ardrone_control_debug')

	try:
		while(True):

			key = getKey()

			hovering_twist = Twist()
			hovering_twist.linear.x = 0; hovering_twist.linear.y = 0; hovering_twist.linear.z = 0
			hovering_twist.angular.x = 0.2; hovering_twist.angular.y = 0; hovering_twist.angular.z = 0

			if key == ' ':
				move_pub.publish(hovering_twist)

			# takeoff and landing
			if key == 'l':
				land_pub.publish(Empty())
			if key == 'r':
				reset_pub.publish(Empty())
			if key == 't':
				takeoff_pub.publish(Empty())
			if key == 'o':
				lock_pub.publish(Empty())
			if key == 'p':
				unlock_pub.publish(Empty())

			if ord(key) == 27:
				key = getKey()
				key = getKey()
			move_twist = Twist()
			target_twist = Twist()
			pid_twist = Twist()
			if ord(key) in move_bindings.keys():
				key = ord(key)
			if key in move_bindings.keys():
				(lin_ang, xyz, speed) = move_bindings[key]
				setattr(getattr(move_twist, lin_ang), xyz, speed)
			else:
				if (key == '\x03'):
					break

			if key in target_bindings.keys():
				(lin_ang, xyz, speed) = target_bindings[key]
				setattr(getattr(target_twist, lin_ang), xyz, speed)
			else:
				if (key == '\x03'):
					break

			if key in pid_bindings.keys():
				(lin_ang, xyz, speed) = pid_bindings[key]
				setattr(getattr(pid_twist, lin_ang), xyz, speed)
				pid_pub.publish(pid_twist)
			else:
				if (key == '\x03'):
					break


			move_pub.publish(move_twist)
			target_pub.publish(target_twist)

	except Exception as e:
		print e
		print repr(e)

	finally:
		move_twist = Twist()
		move_twist.linear.x = 0; move_twist.linear.y = 0; move_twist.linear.z = 0
		move_twist.angular.x = 0; move_twist.angular.y = 0; move_twist.angular.z = 0

		target_twist = Twist()
		target_twist.linear.x = 0; target_twist.linear.y = 0; target_twist.linear.z = 0
		target_twist.angular.x = 0; target_twist.angular.y = 0; target_twist.angular.z = 0

		pid_twist = Twist()
		pid_twist.linear.x = 0; pid_twist.linear.y = 0; pid_twist.linear.z = 0
		pid_twist.angular.x = 0; pid_twist.angular.y = 0; pid_twist.angular.z = 0

		move_pub.publish(move_twist)
		target_pub.publish(target_twist)
		pid_pub.publish(pid_twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


