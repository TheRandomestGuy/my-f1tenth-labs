#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters. Useful variable.
car_width = 0.35
disparity_thres = 0.1
min_gap_depth = 2.5

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)
vis_pub = rospy.Publisher( "/visualization_marker", Marker, queue_size=1)


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
			alter_scan[i] = 5
	for i in disparities:
		min_dist = min(alter_scan[i], alter_scan[i - 1])
		min_dists.append(min_dist)
		num_of_samples.append(int((car_width/(2.0 * (min_dist))) * (len(data.ranges)/(math.pi * 4.0/3.0))))
	for i in range(0, len(disparities)):
		for j in range(max(disparities[i] - num_of_samples[i],0), min(disparities[i] + num_of_samples[i], len(alter_scan) - 1)):
			alter_scan[j] = min(min_dists[i], alter_scan[j])
	return alter_scan

def find_deep_gap_angle(data):
	alter_scan = overwriteDisparities(data)
	max_index = int(math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))
	for i in range(int(math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0))), int(7.0*math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))):
		if alter_scan[max_index] < alter_scan[i]:
			max_index = i
	return max_index/(len(alter_scan)/(math.pi * 4.0/3.0))

def find_wide_gap_angle(data):
	alter_scan = overwriteDisparities(data)
	gap_start = []
	gap_end = []
	curr_gap = []
	widest_start = int(4 * math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))
	widest_end = int(4 * math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))
	for i in range(int(math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0))), int(7.0*math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))):
		if alter_scan[i] > min_gap_depth:
			curr_gap.append(i)
		else:
			if len(curr_gap) > 3:
				gap_start.append(curr_gap[0])
				gap_end.append(curr_gap[-1])
			curr_gap = []
	if len(curr_gap) > 3:
		gap_start.append(curr_gap[0])
		gap_end.append(curr_gap[-1])
	curr_gap = []

	cnt = 0.2

	while len(gap_start) == 0:
		for i in range(int(math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0))), int(7.0*math.pi/6.0 * (len(alter_scan)/(math.pi * 4.0/3.0)))):
			if alter_scan[i] > min_gap_depth - cnt:
				curr_gap.append(i)
			else:
				if len(curr_gap) > 3:
					gap_start.append(curr_gap[0])
					gap_end.append(curr_gap[-1])
				curr_gap = []
		if len(curr_gap) > 3:
			gap_start.append(curr_gap[0])
			gap_end.append(curr_gap[-1])
		curr_gap = []
		cnt += 0.2

	#rospy.loginfo(gap_start)
	#rospy.loginfo(gap_end)
	for i in range(0, len(gap_start)):
		if gap_end[i] - gap_start[i] > widest_end - widest_start:
			widest_start = gap_start[i]
			widest_end = gap_end[i]
	#rospy.loginfo((widest_start)/(len(alter_scan)/(math.pi * 4.0/3.0)))
	#rospy.loginfo((widest_end)/(len(alter_scan)/(math.pi * 4.0/3.0)))
	rospy.loginfo(alter_scan[int(0.5 * (widest_end + widest_start))])
	return (0.5 * (widest_end + widest_start))/(len(alter_scan)/(math.pi * 4.0/3.0))




def callback(data):
	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	#error = find_deep_gap_angle(data) - math.pi * 2.0/3.0
	error = find_wide_gap_angle(data) - math.pi * 2.0/3.0

	arrow = Marker()
	arrow.header.frame_id = "car_8_base_link"
	arrow.header.stamp = rospy.Time()
	arrow.ns = "ns"
	arrow.id = 0
	arrow.type = Marker.ARROW
	arrow.action = Marker.ADD
	arrow.pose.position.x = 0
	arrow.pose.position.y = 0
	arrow.pose.position.z = 0
	arrow.pose.orientation.x = 0
	arrow.pose.orientation.y = 0
	arrow.pose.orientation.z = math.sin(error/2)
	arrow.pose.orientation.w = math.cos(error/2)
	arrow.scale.x = overwriteDisparities(data)[int((error + math.pi * 2.0/3.0) * (len(data.ranges)/(math.pi * 4.0/3.0)))]
	arrow.scale.y = 0.1
	arrow.scale.z = 0.1
	arrow.color.a = 1.0
	arrow.color.r = 0.0
	arrow.color.g = 1.0
	arrow.color.b = 0.0

	msg.pid_error = error
	msg.pid_vel = overwriteDisparities(data)[int(4 * math.pi/6.0 * (len(data.ranges)/(math.pi * 4.0/3.0)))]		# velocity error can also be sent.
	pub.publish(msg)
	vis_pub.publish(arrow)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('gap_scan',anonymous = True)
	# nTODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_8/scan",LaserScan,callback)
	rospy.spin()
