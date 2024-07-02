#!/usr/bin/env python

import rospy 
import random
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute,SetPen
from turtlesim.msg import Pose
from math import pow,atan2,sqrt

def pose_func(msg):
    global pose_yr
    global pose_xr
    pose_yr = msg.y
    pose_xr = msg.x

class ControlTurtlesim():

    def __init__(self):
        rospy.init_node('ControlTurtlesim', anonymous=False)

        rospy.loginfo(" Press CTRL+c to stop moving the Turtle")

        rospy.on_shutdown(self.shutdown)

	sub=rospy.Subscriber('/turtle1/pose', Pose, pose_func)

    	rospy.wait_for_service('/turtle1/set_pen')
    	rospy.wait_for_service('/turtle1/teleport_absolute')
	
	tele = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	sp = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
	tx = random.randrange(3,7)
	ty = random.randrange(3,7)
	sp(off=True)
        tele(tx, ty, 0)
	sp(off=False)
	rospy.loginfo('(%s, %s)'%(tx,ty))

        self.cmd_vel = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=60)
        rate = rospy.Rate(60);
        rospy.loginfo("Set rate 60Hz")

        move_cmd = Twist()

	lin_vx = random.randrange(2,6)
        move_cmd.linear.x = lin_vx	

	ang_vz = random.randrange(1,3)
        move_cmd.angular.z = ang_vz

	temp = 0
	temp2 = 0
	prev = True

	starty = ty
	startx = tx

        while not rospy.is_shutdown():

	    temp = sqrt(pow(pose_xr - startx, 2) + pow(pose_yr - starty, 2))

	    if temp-temp2 > 0 and temp < 0.5 and prev == False:
		ang_vz = -ang_vz

	    prev = temp-temp2 >= 0

	    temp2 = temp

	    move_cmd.angular.z = ang_vz

            self.cmd_vel.publish(move_cmd)

            rate.sleep()


    def shutdown(self):

        rospy.loginfo("Stopping the turtle")

        #self.cmd_vel.publish(Twist())


        rospy.sleep(1)

if __name__ == '__main__':
    try:
        ControlTurtlesim()
    except:
        rospy.loginfo("End of the swim for this Turtle.")
