#!/usr/bin/env python

import rospy 
import random
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute,TeleportRelative,SetPen
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,pi

def pose_func(msg):
    global pose_yb
    global pose_xb
    pose_yb = msg.y
    pose_xb = msg.x

class ControlTurtlesim():

    def __init__(self):
        rospy.init_node('ControlTurtlesim', anonymous=False)

        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")

        rospy.on_shutdown(self.shutdown)

	rospy.wait_for_service('/reset')
	res = rospy.ServiceProxy('/reset', Empty)
	res()

	sub=rospy.Subscriber('/turtle1/pose', Pose, pose_func)

    	rospy.wait_for_service('/turtle1/set_pen')
    	rospy.wait_for_service('/clear')
    	rospy.wait_for_service('/turtle1/teleport_absolute')
    	rospy.wait_for_service('/turtle1/teleport_relative')
	
	tele = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	teler = rospy.ServiceProxy('/turtle1/teleport_relative', TeleportRelative)
	sp = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
	cbg = rospy.ServiceProxy('/clear', Empty)

	sp(off=True)
        tele(1,1, 0)
	sp(off=False)
	cbg = rospy.ServiceProxy('/clear', Empty)

	rospy.set_param('/turtlesim/background_b', 0)
	rospy.set_param('/turtlesim/background_g', 0)
	rospy.set_param('/turtlesim/background_r', 255)
	cbg()

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=60)
        rate = rospy.Rate(60);
        rospy.loginfo("Set rate 60Hz")

	side_l = input('Enter Side Length (in range 1 to 5): ')

        move_cmd = Twist()

        move_cmd.linear.x = 1	

        move_cmd.angular.z = 0

	sx = 1
	sy = 1
	temp = 0

        while not rospy.is_shutdown():
	    
	    temp = sqrt(pow(pose_xb - sx, 2) + pow(pose_yb - sy,2))

	    if temp > side_l:
		sx = pose_xb
		sy = pose_yb
		teler(0,pi/2)
	    
            self.cmd_vel.publish(move_cmd)

            rate.sleep()


    def shutdown(self):

        rospy.loginfo("Stopping the turtle")
	rospy.set_param('/turtlesim/background_b', 255)
	rospy.set_param('/turtlesim/background_g', 86)
	rospy.set_param('/turtlesim/background_r', 69)
    	rospy.wait_for_service('/clear')
	cbg = rospy.ServiceProxy('/clear', Empty)
	cbg()

        self.cmd_vel.publish(Twist())


        rospy.sleep(1)

if __name__ == '__main__':
    try:
        ControlTurtlesim()
    except:
        rospy.loginfo("End of the swim for this Turtle.")
