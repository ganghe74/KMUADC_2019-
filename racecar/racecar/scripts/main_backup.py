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
from obstacle_detector.msg import Obstacles
from cv_bridge import CvBridge, CvBridgeError

from ackermann_msgs.msg import AckermannDriveStamped
from datetime import datetime # for record

from Stop_Counter import Stop_Counter
from CurveDetector import CurveDetector

x_location_old = None

warper = Warper()
slidewindow  = SlideWindow()
pidcal = PidCal()

q1 = que.Queue()
bridge = CvBridge()

now = datetime.now() # for record

cv_image = None
obstacles = None
ack_publisher = None
car_run_speed = 1.5


def img_callback(data):
    global cv_image
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

def obstacle_callback(data):
	global obstacles
	obstacles = data
  
def auto_drive(pid):
    global car_run_speed
    #w = 0
    #if -0.065 < pid and pid < 0.065:
    #    w = 1
    #else:
    #    w = 0.3

    #if car_run_speed < 1.0 * w:
    #    car_run_speed += 0.002 * 10
    #else:
    #    car_run_speed -= 0.003 * 10
    
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = ''
    ack_msg.drive.steering_angle = pid
    ack_msg.drive.speed = car_run_speed
    ack_publisher.publish(ack_msg)
    print 'speed: ' 
    print car_run_speed 

def main():
    global cv_image
    global ack_publisher
    global x_location_old
    global car_run_speed
    pid = None
    
    rospy.sleep(3)
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    obstacle_sub = rospy.Subscriber("/obstacles", Obstacles, obstacle_callback, queue_size = 1)
    
    rospy.init_node('auto_xycar', anonymous=True)

    #ack_publisher = rospy.Publisher('vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    ack_publisher = rospy.Publisher('ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    # record the processed
    out = cv2.VideoWriter('/home/nvidia/Desktop/video/processed {}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    # record the origin
    out2 = cv2.VideoWriter('/home/nvidia/Desktop/video/original {}-{}-{} {}-{}.avi'.format(now.year, now.month, now.day, now.hour, now.minute), cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    stop_counter = Stop_Counter()
    curve_detector = CurveDetector()

    while not rospy.is_shutdown():
      img1, x_location = process_image(cv_image)
      
      if x_location != None:
          x_location_old = x_location
          pid = round(pidcal.pid_control(int(x_location)), 6)
          print pid
          auto_drive(pid)
      else:
          pid = round(pidcal.pid_control(int(x_location_old)), 6)
          print pid
          auto_drive(pid)

      curve_detector.list_update(pid)
      curve_detector.count_curve()

      # mode on
      if curve_detector.curve_count == 2:          
          car_run_speed = 1.0
	  #for circle in obstacles.circles:
	  	#p = circle.center
	  	#if abs(p.x) <= -0.2 and -1 <= p.y <=5:
	  		#ack_msg.drive.steering_angle -=0.05
			#print("!!!!")

      detected = stop_counter.check_stop_line(cv_image)
      if detected: # stop line detected
          curve_detector.curve_count = 0
          car_run_speed = 1.5
      if stop_counter.cnt == 3: # finish
          break

      cv2.imshow('result', img1)
      cv2.imshow("origin", cv_image)
      cv2.putText(img1, 'PID %f'%pid, (0,15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
      
      out.write(img1)
      out2.write(cv_image)

      if cv2.waitKey(1) & 0xFF == ord('q'):
          break

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
