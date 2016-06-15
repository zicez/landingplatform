#!/usr/bin/env python
import roslib; roslib.load_manifest('autoland_pid')
import rospy
from geometry_msgs.msg import Twist


def control_reset():

    while not rospy.is_shutdown():
        hovering_twist = Twist()
        move_pub.publish(hovering_twist)

        rospy.sleep(0.24)

if __name__ == '__main__':

    move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('ardrone_control_reset')

    try:
        control_reset()
    except rospy.ROSInterruptException: pass
