#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
car_width = 0.65
disparity_thres = 0.1

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def findDisparities(data):
	global disparity_thres
	disparities = []

	prev = data.ranges[int(math.pi/6.0 * (len(data.ranges)/(math.pi * 4.0/3.0)))]

	for i in range(int(math.pi/6.0 * (len(data.ranges)/(math.pi * 4.0/3.0))), int(7.0*math.pi/6.0 * (len(data.ranges)/(math.pi * 4.0/3.0)))):
		if (math.isnan(data.ranges[i]) and not math.isnan(prev)) or abs(prev - data.ranges[i]) > disparity_thres: disparities.append(i)
		prev = data.ranges[i]

	return disparities
	
def overwriteDisparities(data):
	disparities = findDisparities(data)
	num_of_samples = []
	min_dists = []
	alter_scan = list(data.ranges)

	for i in range(0, len(data.ranges)):
		if math.isnan(data.ranges[i]):
			alter_scan[i] = 7.0
		if data.ranges[i] < 0.05:
			disparities.append(i)
			alter_scan[i] = 4
	for i in disparities:
		min_dist = min(alter_scan[i], alter_scan[i - 1])
		min_dists.append(min_dist)
		num_of_samples.append(int((car_width/(2.0 * min_dist)) * (len(data.ranges)/(math.pi * 4.0/3.0))))
	for i in range(0, len(disparities)):
		for j in range(max(disparities[i] - num_of_samples[i],0), min(disparities[i] + num_of_samples[i], len(alter_scan) - 1)):
			alter_scan[j] = min(min_dists[i], alter_scan[j])
	return alter_scan

def find_gap_angle(data):
	alter_scan = overwriteDisparities(data)
	max_index = int(math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))
	for i in range(int(math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0))), int(7.0*math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))):
		if alter_scan[max_index] < alter_scan[i]:
			max_index = i
	return max_index/(len(alter_scan)/(math.pi * 4.0/3.0))




def callback(data):
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	error = find_gap_angle(data) - math.pi * 2.0/3.0
	msg.pid_error = error
	msg.pid_vel = min(2.0,overwriteDisparities(data)[int(4 * math.pi/6.0 * (len(data.ranges)/(math.pi * 4.0/3.0)))])/2		# velocity error can also be sent.
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('gap_scan',anonymous = True)
	# nTODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_8/scan",LaserScan,callback)
	rospy.spin()
