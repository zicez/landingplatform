#!/usr/bin/env python
import roslib; roslib.load_manifest('quadcopter_control')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

def Timer():
    
    timer_pub = rospy.Publisher('/pid_timer', Empty, queue_size=1)
    rospy.init_node('psulu_timer')
    
    while not rospy.is_shutdown():     
	    rospy.sleep(0.05)
	    timer_pub.publish()


if __name__=="__main__":

	try:
		Timer()
	except rospy.ROSInterruptException: pass

        






