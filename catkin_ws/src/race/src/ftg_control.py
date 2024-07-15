#!/usr/bin/env python
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive

# PID Control Params

kp = 20.0 #TODO
kd = 0.3 #TODO
ki = 0.0 #TODO
servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.
prev_error = 0.0


# This code can input desired velocity from the user.
# velocity must be between [0,100] to move forward.
# The following velocity values correspond to different speed profiles.
# 15: Very Slow (Good for debug mode)
# 25: Slow and steady
# 35: Nice Autonomous Pace
# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
vel_input = 10.0	#nTODO

# Publisher for moving the car.
# nTODO: Use the coorect topic /car_x/offboard/command. The multiplexer listens to this topic
command_pub = rospy.Publisher('/car_8/multiplexer/command', AckermannDrive, queue_size = 1)

def control(data):
	global prev_error
	global vel_input
	global kp
	global kd
	global angle

	#print("PID Control Node is Listening to error")

	## Your PID code goes here
	#pTODO: Use kp, ki & kd to implement a PID controller
	# 1. Scale the error

	error = 2 * data.pid_error/math.pi * 10
	# 2. Apply the PID equation on error to compute steering
	steer_corr = kp * error + kd * (prev_error - error)
	prev_error = error
	# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
	command = AckermannDrive()

	# nTODO: Make sure the steering value is within bounds [-100,100]
	if data.pid_vel < 0.25:
		steer_corr *= 100
	command.steering_angle = min(max(-steer_corr, -100), 100)
	#command.steering_angle = -steer_corr

	# nTODO: Make sure the velocity is within bounds [0,100]
	vel_i = min(2,data.pid_vel - 0.1)/2
	vel_f = vel_input * 0.35 + (vel_input * 0.65 * math.pow((10 - min(abs(error), 10))/10, 2))
	if error == 0.0:
		command.speed = min(max(vel_f * (0.5 * vel_i + 0.5) * 0.6, 0), 100)
	else:	
		command.speed = min(max(vel_f * (0.5 * vel_i + 0.5), vel_input * 0.3), 100)

	# Move the car autonomously
	command_pub.publish(command)

if __name__ == '__main__':

    # This code tempalte asks for the values for the gains from the user upon start, but you are free to set them as ROS parameters as well.
	#global kp
	#global kd
	#global ki
	global vel_input
	#kp = input("Enter Kp Value: ")
	#kd = input("Enter Kd Value: ")
	#ki = input("Enter Ki Value: ")
	vel_input = input("Enter desired velocity: ")
	rospy.init_node('pid_controller', anonymous=True)
    # subscribe to the error topic
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
