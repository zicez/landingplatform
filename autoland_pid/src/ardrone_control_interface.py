#!/usr/bin/env python2
import roslib; roslib.load_manifest('autoland_pid')
import rospy
import struct

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from socket import *

host = "192.168.1.1"
#AT_port = 50000
port_AT = 5556
port_nav = 5554
port_video = 5555
AT_addr = (host,port_AT)
nav_addr = (host,port_nav)
video_addr = (host,port_video)
AT_bug = 1024

def handle_takeoff(msg):

    UDPSock = socket(AF_INET,SOCK_DGRAM)

    print 'taking off\r'
    altitude = "AT*CONFIG=%d,control:altitude_max,6000\r"
    data = "AT*REF=%d,290718208\r" % (1)

    #print data
    UDPSock.sendto(altitude,AT_addr)
    UDPSock.sendto(data,AT_addr)

def handle_land(msg):

    UDPSock = socket(AF_INET,SOCK_DGRAM)

    print 'landing\r'

    data = "AT*REF=%d,290717696\r" % (1)

    #print data
    UDPSock.sendto(data,AT_addr)

def handle_reset(msg):

    UDPSock = socket(AF_INET,SOCK_DGRAM)

    print 'reset\r'

    data = "AT*REF=%d,290717952\r" % (1)

    #print data
    UDPSock.sendto(data,AT_addr)

def handle_move(msg):

    mx = struct.unpack('i', struct.pack('f', -1*msg.linear.x))[0]
    my = struct.unpack('i', struct.pack('f', -1*msg.linear.y))[0]
    mz = struct.unpack('i', struct.pack('f', msg.linear.z))[0]

    rx = struct.unpack('i', struct.pack('f', msg.angular.x))[0]
    ry = struct.unpack('i', struct.pack('f', msg.angular.y))[0]
    rz = struct.unpack('i', struct.pack('f', -1*msg.angular.z))[0]

    # print 'rx: ',msg.angular.x,'\r'
    # print 'rz: ',msg.angular.z,'\r'

    command = "AT*PCMD=%d,1,%d,%d,%d,%d\r" % (1, my, mx, mz, rz)

    hovering = "AT*PCMD=%d,1,%d,%d,%d,%d\r" % (1, 0, 0, 0, 0)


    # print command

    # print 'Move:   ',mx,' ',my,' ',mz
    # print 'Rotate: ',rx,' ',ry,' ',rz

    UDPSock = socket(AF_INET,SOCK_DGRAM)
    UDPSock.sendto(command,AT_addr)
#rospy.sleep(0.02)
#UDPSock.sendto(hovering,AT_addr)





if __name__ == "__main__":

    global sequence
    sequence = 0

    rospy.init_node('quadcopter_control_interface', anonymous=True)

    rospy.Subscriber("/ardrone/takeoff", Empty, handle_takeoff)
    rospy.Subscriber("/ardrone/land", Empty, handle_land)
    rospy.Subscriber("/ardrone/reset", Empty, handle_reset)
    rospy.Subscriber("/cmd_vel", Twist, handle_move)

    ATSock = socket(AF_INET, SOCK_DGRAM)
    data = "AT*COMWDG=1\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)
    data = "AT*REF=1,290717696\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)
    data = "AT*PCMD=1,0,0,0,0,0\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)
    data = "AT*REF=1,290717696\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)
    data = "AT*REF=1,290717696\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)
    data = "AT*CONFIG=605,\"detect:detect_type\",\"10\"\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)
    data = "AT*CONFIG=605,\"detect:detections_select_v\",\"0\"\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)
    data = "AT*CONFIG=605,\"detect:detections_select_h\",\"3\"\r"
    ATSock.sendto(data, AT_addr)
    rospy.sleep(0.1)

    trigger = "\x01\x00\x00\x00"
    ATSock.sendto(trigger, nav_addr)
    rospy.sleep(0.1)


    rospy.spin()
