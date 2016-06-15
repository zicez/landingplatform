#!/usr/bin/env python
import roslib; roslib.load_manifest('autoland_pid')
import rospy
import struct
import socket
#import arvideo
#import pygame
from sensor_msgs.msg import Image as SensorImage
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

#import math
#import numpy as np
from collections import namedtuple
import numpy as np

remotehost = "192.168.1.1"
port_nav = 5554
port_video = 5555
port_AT = 5556
AT_addr = (remotehost,port_AT)
nav_addr = (remotehost,port_nav)
video_addr = (remotehost,port_video)




def decode_navdata(packet):
    """Decode a navdata packet."""
    offset = 0
    _ = struct.unpack_from("IIII", packet, offset)
    drone_state = dict()
    drone_state['fly_mask'] = _[1] & 1 # FLY MASK : (0) ardrone is landed, (1) ardrone is flying
    drone_state['video_mask'] = _[1] >> 1 & 1 # VIDEO MASK : (0) video disable, (1) video enable
    drone_state['vision_mask'] = _[1] >> 2 & 1 # VISION MASK : (0) vision disable, (1) vision enable */
    drone_state['control_mask'] = _[1] >> 3 & 1 # CONTROL ALGO (0) euler angles control, (1) angular speed control */
    drone_state['altitude_mask'] = _[1] >> 4 & 1 # ALTITUDE CONTROL ALGO : (0) altitude control inactive (1) altitude control active */
    drone_state['user_feedback_start'] = _[1] >> 5 & 1 # USER feedback : Start button state */
    drone_state['command_mask'] = _[1] >> 6 & 1 # Control command ACK : (0) None, (1) one received */
    drone_state['fw_file_mask'] = _[1] >> 7 & 1 # Firmware file is good (1) */
    drone_state['fw_ver_mask'] = _[1] >> 8 & 1 # Firmware update is newer (1) */
    drone_state['fw_upd_mask'] = _[1] >> 9 & 1 # Firmware update is ongoing (1) */
    drone_state['navdata_demo_mask'] = _[1] >> 10 & 1 # Navdata demo : (0) All navdata, (1) only navdata demo */
    drone_state['navdata_bootstrap'] = _[1] >> 11 & 1 # Navdata bootstrap : (0) options sent in all or demo mode, (1) no navdata options sent */
    drone_state['motors_mask'] = _[1] >> 12 & 1 # Motor status : (0) Ok, (1) Motors problem */
    drone_state['com_lost_mask'] = _[1] >> 13 & 1 # Communication lost : (1) com problem, (0) Com is ok */
    drone_state['vbat_low'] = _[1] >> 15 & 1 # VBat low : (1) too low, (0) Ok */
    drone_state['user_el'] = _[1] >> 16 & 1 # User Emergency Landing : (1) User EL is ON, (0) User EL is OFF*/
    drone_state['timer_elapsed'] = _[1] >> 17 & 1 # Timer elapsed : (1) elapsed, (0) not elapsed */
    drone_state['angles_out_of_range'] = _[1] >> 19 & 1 # Angles : (0) Ok, (1) out of range */
    drone_state['ultrasound_mask'] = _[1] >> 21 & 1 # Ultrasonic sensor : (0) Ok, (1) deaf */
    drone_state['cutout_mask'] = _[1] >> 22 & 1 # Cutout system detection : (0) Not detected, (1) detected */
    drone_state['pic_version_mask'] = _[1] >> 23 & 1 # PIC Version number OK : (0) a bad version number, (1) version number is OK */
    drone_state['atcodec_thread_on'] = _[1] >> 24 & 1 # ATCodec thread ON : (0) thread OFF (1) thread ON */
    drone_state['navdata_thread_on'] = _[1] >> 25 & 1 # Navdata thread ON : (0) thread OFF (1) thread ON */
    drone_state['video_thread_on'] = _[1] >> 26 & 1 # Video thread ON : (0) thread OFF (1) thread ON */
    drone_state['acq_thread_on'] = _[1] >> 27 & 1 # Acquisition thread ON : (0) thread OFF (1) thread ON */
    drone_state['ctrl_watchdog_mask'] = _[1] >> 28 & 1 # CTRL watchdog : (1) delay in control execution (> 5ms), (0) control is well scheduled */
    drone_state['adc_watchdog_mask'] = _[1] >> 29 & 1 # ADC Watchdog : (1) delay in uart2 dsr (> 5ms), (0) uart2 is good */
    drone_state['com_watchdog_mask'] = _[1] >> 30 & 1 # Communication Watchdog : (1) com problem, (0) Com is ok */
    drone_state['emergency_mask'] = _[1] >> 31 & 1 # Emergency landing : (0) no emergency, (1) emergency */
    data = dict()
    data['drone_state'] = drone_state
    data['header'] = _[0]
    data['seq_nr'] = _[2]
    data['vision_flag'] = _[3]
    offset += struct.calcsize("IIII")
    while 1:
        try:
            id_nr, size = struct.unpack_from("HH", packet, offset)
            offset += struct.calcsize("HH")
        except struct.error:
            break
        values = []
        for i in range(size-struct.calcsize("HH")):
            values.append(struct.unpack_from("c", packet, offset)[0])
            offset += struct.calcsize("c")
        # navdata_tag_t in navdata-common.h
        if id_nr == 0:
            values = struct.unpack_from("IIfffIfffI", "".join(values))
            values = dict(zip(['ctrl_state', 'battery', 'theta', 'phi', 'psi', 'altitude', 'vx', 'vy', 'vz', 'num_frames'], values))
            # convert the millidegrees into degrees and round to int, as they
            # are not so precise anyways
            for i in 'theta', 'phi', 'psi':
                values[i] = int(values[i] / 1000)
            #values[i] /= 1000
        data[id_nr] = values
    return data

def surf2CV(surf):
    """
    Given a Pygame surface, convert to an OpenCv cvArray format.
    Either Ipl image or cvMat.
    """
    numpyImage = pygame.surfarray.pixels3d(surf)#.copy()    # Is this required to be a copy?
    cvImage = cv.adaptors.NumPy2Ipl(numpyImage.transpose(1,0,2))
    return cvImage


if __name__ == "__main__":

    rospy.init_node('ardrone_nav_interface', anonymous=True)

    bridge = CvBridge()

    ATSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    ATSock.bind(('',port_AT))
    ATSock.settimeout(0.25)
    NAVSock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    NAVSock.bind(('',port_nav))
    NAVSock.settimeout(0.25)
    VideoSock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    VideoSock.connect((remotehost,port_video))
    VideoSock.settimeout(0.25)

    while not rospy.is_shutdown():
        #data = "AT*PMODE=1,2\r"
        #UDPSock.sendto(data,AT_addr)
        #rospy.sleep(0.1)
        #data = "AT*MISC=1,2,20,2000,3000\r"
        #UDPSock.sendto(data,AT_addr)
        #rospy.sleep(0.1)
        #data = "AT*REF=1,290717696\r"
        #UDPSock.sendto(data,AT_addr)
        #rospy.sleep(0.1)
        data = "AT*COMWDG=1\r"
        ATSock.sendto(data,AT_addr)
        rospy.sleep(0.1)
        #data = "AT*CONFIG=1,\"control:altitude_max\",\"2000\"\r"
        #print data
        #UDPSock.sendto(data,AT_addr)
        #rospy.sleep(0.1)
        #data = "AT*CONFIG=1,\"control:control_level\",\"0\"\r"
        #UDPSock.sendto(data,AT_addr)
        #rospy.sleep(0.1)
        # data = "AT*CONFIG=1,\"general:navdata_demo\",\"TRUE\"\r"
        # ATSock.sendto(data,AT_addr)
        # rospy.sleep(0.1)
        # data = "AT*CTRL=0\r"
        # ATSock.sendto(data,AT_addr)
        # rospy.sleep(0.1)
        # data = "AT*CONFIG=1,\"general:video_enable\",\"TRUE\"\r"
        # ATSock.sendto(data,AT_addr)
        # rospy.sleep(0.1)
        #data = "AT*CONFIG=1,\"pic:ultrasound_freq\",\"8\"\r"
        #UDPSock.sendto(data,AT_addr)
        #rospy.sleep(0.1)
        data = "AT*REF=1,290717696\r"
        ATSock.sendto(data,AT_addr)
        rospy.sleep(0.1)
        data = "AT*PCMD=1,0,0,0,0,0\r"
        ATSock.sendto(data,AT_addr)
        rospy.sleep(0.1)
        data = "AT*REF=1,290717696\r"
        ATSock.sendto(data,AT_addr)
        rospy.sleep(0.1)
        data = "AT*REF=1,290717696\r"
        ATSock.sendto(data,AT_addr)
        rospy.sleep(0.1)
        trigger = "\x01\x00\x00\x00"
        NAVSock.sendto(trigger,nav_addr)
        rospy.sleep(0.1)
        VideoSock.send(trigger)
        rospy.sleep(0.1)
        # data = "AT*CONFIG=1,\"general:navdata_demo\",\"TRUE\"\r"
        # UDPSock.sendto(data,AT_addr)
        # rospy.sleep(0.5)
        # data = "AT*CONFIG=1,\"general:video_enable\",\"TRUE\"\r"
        # UDPSock.sendto(data,AT_addr)
        # rospy.sleep(0.5)
        # data = "AT*CONFIG=1,\"general:vision_enable\",\"TRUE\"\r"
        # UDPSock.sendto(data, AT_addr)
        # rospy.sleep(0.5)
        # data = "AT*CONFIG=605,\"video:video_codec\",\"131\"\r"
        # ATSock.sendto(data, AT_addr)
        # rospy.sleep(0.1)
        # data = "AT*CONFIG=605,\"video:video_channel\",\"1\"\r"
        # ATSock.sendto(data, AT_addr)
        # rospy.sleep(0.1)
        data = "AT*CONFIG=605,\"detect:detect_type\",\"10\"\r"
        ATSock.sendto(data, AT_addr)
        rospy.sleep(0.1)
        data = "AT*CONFIG=605,\"detect:detections_select_v\",\"0\"\r"
        ATSock.sendto(data, AT_addr)
        rospy.sleep(0.1)
        data = "AT*CONFIG=605,\"detect:detections_select_h\",\"3\"\r"
        ATSock.sendto(data, AT_addr)
        rospy.sleep(0.1)

        message, addr = NAVSock.recvfrom( 65535 ) # buffer size is 1024 bytes

        try:
            nav_message_decoded = decode_navdata(message)[0]
            print "NAV Data Ready!\n\r"
            break
        except KeyError:
            print "NAV Data Not Ready!\n\r"

    # image_pub = rospy.Publisher('image_raw', SensorImage, queue_size=1)
    # commented_image_pub = rospy.Publisher('image_raw_commented', SensorImage, queue_size=1)
    #cv.NamedWindow("UAV Video Stream", 0)
    #cv.ResizeWindow("UAV Video Stream", 640, 480)
    # Image = namedtuple('Image', ('width', 'height', 'frame_nr', 'data'))

    while not rospy.is_shutdown():
        #switch_count = switch_count + 1

        #if (switch_count > 100):
        #	switch_count = 0
        #	data = "AT*CONFIG=1,\"video:video_channel\",\"1\"\r"
        #	UDPSock.sendto(data,AT_addr)
        #	rospy.sleep(0.1)

        try:
            nav_data, addr = NAVSock.recvfrom( 65535 ) # buffer size is 1024 bytes
        except socket.error:
            print "Navdata Timeout\n\r"
            NAVSock.sendto(trigger,nav_addr)
        print 'Received nav size: ',str(len(nav_data)),'\r'

        # try:
        #     image_data = VideoSock.recv( 655350 ) # buffer size is 1024 bytes
        # except socket.error:
        #     print "Video Timeout\n\r"
        #     VideoSock.sendto(trigger,video_addr)
        #
        # print 'Received image size: ',str(len(image_data)),'\r'

        if len(nav_data) > 100:
            nav_message_decoded = decode_navdata(nav_data)[0]

            # image = Image(yuv)
            # image_cv_original = cv.CreateMat(image.height,image.width,cv.CV_8UC4 )
            # image_cv = cv.CreateMat (image.height,image.width,cv.CV_8UC4 )
            # #print "width ",image.width," height ",image.height," frame nr: ",image.frame_nr
            # cv.SetData(image_cv_original,image.data)
            # #cv.CvtColor(image_cv_original,image_cv,cv.CV_BGRA2RGBA)
            # cv.CvtColor(image_cv_original,image_cv,cv.CV_BGRA2RGBA)
            #
            # image_msg = bridge.cv_to_imgmsg(image_cv, "bgra8")
            # image_pub.publish(image_msg)
            # # Show nav data on the image
            # (cols,rows) = cv.GetSize(image_cv)
            status1 = 'Yaw: '+str(round(nav_message_decoded['psi'], 4)) + ' Pitch: ' + str(round(nav_message_decoded['theta'], 4)) + ' Roll: ' + str(round(nav_message_decoded['phi'], 4))
            status2 = 'Altitude: '+str(round(nav_message_decoded['altitude'], 4)) + ' Battery: ' + str(round(nav_message_decoded['battery'], 4))
            status3 = 'Velocity X: '+str(nav_message_decoded['vx']) + ' Y: ' + str(nav_message_decoded['vy']) + ' Z: ' + str(nav_message_decoded['vz'])

            print(status1)
            print(status2)
            print(status3)

            #
            # font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 0.4, 0.3, 0, 1, 8)
            # cv.PutText(image_cv,status1,(5,rows - 25),font,(0,255,255));
            # cv.PutText(image_cv,status2,(5,rows - 15),font,(0,255,255));
            # cv.PutText(image_cv,status3,(5,rows - 5),font,(0,255,255));
            #
            # image_msg_commented = bridge.cv_to_imgmsg(image_cv, "bgra8")
            # commented_image_pub.publish(image_msg_commented)
            #cv.WaitKey(3)
            #cv.ShowImage("UAV Video Stream", image_cv)







        rospy.sleep(0.03)

    cap.release()
    cv.destroyAllWindows()

    rospy.spin()
