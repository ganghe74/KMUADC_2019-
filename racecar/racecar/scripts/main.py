#!/usr/bin/env python
import cv2
import threading
import Queue as que
import time
import numpy as np

import roslib
import sys
import rospy

import importlib
import cPickle
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from rosservice import ROSServiceException

from slidewindow import SlideWindow
from warper import Warper
from pidcal import PidCal

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from ackermann_msgs.msg import AckermannDriveStamped
from datetime import datetime # for record

from Stop_Counter import Stop_Counter
from CurveDetector import CurveDetector

warper = Warper()
slidewindow  = SlideWindow()
pidcal = PidCal()

q1 = que.Queue()
bridge = CvBridge()

now = datetime.now() # for record

cv_image = None
ack_publisher = None
car_run_speed = 1.2

def img_callback(data):
    global cv_image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)
  
def auto_drive(pid):
    global car_run_speed
    
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = ''
    ack_msg.drive.steering_angle = pid
    ack_msg.drive.speed = car_run_speed
    ack_publisher.publish(ack_msg)
    #print 'speed: ' 
    #print car_run_speed 

def main():
    global cv_image
    global ack_publisher
    global car_run_speed

    rospy.sleep(3)
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    
    rospy.init_node('auto_xycar', anonymous=True)

    #ack_publisher = rospy.Publisher('vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    ack_publisher = rospy.Publisher('ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    
    # record the processed
    out = cv2.VideoWriter('/home/nvidia/Desktop/video/processed {}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour, now.minute),cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    # record the origin
    out2 = cv2.VideoWriter('/home/nvidia/Desktop/video/original {}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour, now.minute),cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    stop_counter = Stop_Counter()
    curve_detector = CurveDetector()

    while not rospy.is_shutdown():
        img1, x_location = process_image(cv_image)
        cv2.imshow('result', img1)
        if x_location != None:
            pid = round(pidcal.pid_control(int(x_location)), 6)
            curve_detector.list_update(pid)
            curve_detector.count_curve()
            
            # mode on
            if curve_detector.curve_count == 2:
                car_run_speed = 0.7

            print pid
            auto_drive(pid)

        detected = stop_counter.check_stop_line(cv_image)
	if detected: # stop line detected
	        curve_detector.curve_count = 0
    	if stop_counter.cnt == 3: # finish
	        break
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        cv2.putText(img1, 'PID %f'%pid, (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.imshow("origin", cv_image)
        out.write(img1)
        out2.write(cv_image)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
	print('Finally')
        out.release()
        out2.release()
        cv2.destroyAllWindows() 

def process_image(frame):
    
    # grayscle
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # canny edge
    low_threshold = 60#60
    high_threshold = 70# 70
    edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # warper
    img = warper.warp(edges_img)
    img1, x_location = slidewindow.slidewindow(img)
    
    return img1, x_location

if __name__ == '__main__':
    main()
