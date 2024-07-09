#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
car_width = 0.30
disparity_thres = 0.1

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def findDisparities(data):
	global disparity_thres
	disparities = []

	prev = data[int(30 * (len(data.ranges)/(math.pi * 4/3))) - 1]

	for i in range(int(30 * (len(data.ranges)/(math.pi * 4/3))), int(210 * (len(data.ranges)/(math.pi * 4/3)))):
		if abs(prev - data.ranges[i]) > disparity_thres: disparities.append(i)
		prev = data.ranges[i]

	return disparities
	
def overwriteDisparities(data):
	disparities = findDisparities(data)
	num_of_samples = []
	min_dists = []
	alter_scan = list(data.ranges).copy()
	for i in disparities:
		min_dist = min(alter_scan[i], alter_scan[i - 1])
		min_dists.append(min_dist)
		num_of_samples.append(int((car_width/(2 * min_dist)) * (len(data.ranges)/(math.pi * 4/3))))
	for i in range(0, len(disparities)):
		for j in range(disparities[i] - num_of_samples[i], disparities[i] + num_of_samples[i]):
			alter_scan[j] = min(min_dists[i], alter_scan[j])
	return alter_scan

def find_gap_angle(data):
	alter_scan = overwriteDisparities(data)
	max_index = int(30 * (len(alter_scan)/(math.pi * 4/3)))
	for i in range(int(30 * (len(alter_scan)/(math.pi * 4/3))), int(210 * (len(alter_scan)/(math.pi * 4/3)))):
		if alter_scan[max_index] < alter_scan[i]:
			max_index = i
	return max_index/(len(alter_scan)/(math.pi * 4/3))




def callback(data):
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	error = find_gap_angle(data) - 120
	msg.pid_error = error
	msg.pid_vel = 15		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('gap_scan',anonymous = True)
	# nTODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_8/scan",LaserScan,callback)
	rospy.spin()
