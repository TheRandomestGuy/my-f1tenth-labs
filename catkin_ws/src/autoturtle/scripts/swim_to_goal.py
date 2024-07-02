#!/usr/bin/env python

import rospy 
import random
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute,TeleportRelative,SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,pi

def pose_func(msg):
    global pose_ys
    global pose_xs
    global pose_ts
    pose_ys = msg.y
    pose_xs = msg.x
    pose_ts = msg.theta % (2 * pi)

class ControlTurtlesim():

    def __init__(self):
        rospy.init_node('ControlTurtlesim', anonymous=False)

        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")

        rospy.on_shutdown(self.shutdown)

	rospy.wait_for_service('/reset')
	res = rospy.ServiceProxy('/reset', Empty)
	res()

	sub=rospy.Subscriber('/turtle1/pose', Pose, pose_func)

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=60)
        rate = rospy.Rate(10);
        rospy.loginfo("Set rate 60Hz")

	xc = input('Enter X Coordinate (in range 1 to 9): ')
	yc = input('Enter Y Coordinate (in range 1 to 9): ')
	tol = input('Enter Tolerance Value (in range 0.1 to 1): ')

        move_cmd = Twist()

	lin = 1
	ang = 1	
	v_gain = 0.75
	a_gain = 2

        move_cmd.linear.x = lin	

        move_cmd.angular.z = ang

	d = 0
	da = 0

        while not rospy.is_shutdown():
	    
	    d = sqrt(pow(pose_xs - xc, 2) + pow(pose_ys - yc,2))
	    da = atan2(pose_ys - yc, pose_xs-xc) % pi
	    if(pose_ys > yc): da += pi	    
	    
	    da = (da - pose_ts + pi) % (2*pi) - pi

	    lin = v_gain * d
	    ang = a_gain * da
	    
	    move_cmd.linear.x = lin	
            move_cmd.angular.z = ang

            self.cmd_vel.publish(move_cmd)

	    if d < tol:
        	self.cmd_vel.publish(Twist())

            rate.sleep()


    def shutdown(self):

        rospy.loginfo("Stopping the turtle")

        self.cmd_vel.publish(Twist())

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        ControlTurtlesim()
    except:
        rospy.loginfo("End of the swim for this Turtle.")
