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
	flag = False
	for circle in obstacles.circles:
		p = circle.center
		if abs(p.x) <= 0.3 and -1 <= p.y <= 0:
			flag = True
	if flag:
		print("DETECT")
	else:
		print("no")

	time.sleep(1)
