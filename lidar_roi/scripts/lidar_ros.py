#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import LaserScan
from obstacle_detector.msg import Obstacles

obstacles = None

def callback(data):
	global obstacles
	obstacles = data

rospy.init_node('test')
rospy.Subscriber('/obstacles', Obstacles, callback, queue_size = 1)

time.sleep(3)

while not rospy.is_shutdown():
	flag = 0
	for circle in obstacles.circles:
		p = circle.center
		if -0.4 <= p.x <= -0.15 and -1 <= p.y <= 0:
			flag = 1
		elif 0.15 <= p.x <= 0.4 and -1 <= p.y <= 0:
			flag = 2

	if flag == 1:
		print("LEFT OBSTACLE DETECT!!")
	elif flag == 2:
		print("RIGHT OBSTACLE DETECT!!")
	else:
		print("NONE")

	time.sleep(1)
