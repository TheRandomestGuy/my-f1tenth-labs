# F1Tenth Assignments
## Autonomous Turtle
Before working with a F1Tenth car, I completed an assignment using TurtleSim, a simple simulator. The scripts from this assignment can be found in the autoturtle package.

While working on this assignment I learned the following:
- Basic ROS Commands
- Creating ROS Nodes
- Creating, subscribing to, and publishing to ROS Topics
- Creating ROS Messages
- Calling ROS Services
- Setting ROS Params
- Creating ROS Launch Files

### Swim School
This script allows you to enter a linear velocity and angular velocity. The turtle then creates a figure 8 using those values. In order to achieve this, a node must be created to subscribe to the pose topic of the turtle and publish a Twist message on cmd_vel topic.
### Random Swim School
This script makes the turtle create a figure 8 with a random angular velocity and random linear velocity at a random location with no other line traces. This builds off the previous script but also requires calling the teleport_absolute and set_pen services.
### Back to Square One
This script allows the user to enter an integer, which will be the side length of a square with a corner at (1, 1). Additionally the background must turn red. This buids off the previous scripts but also requires calling the teleport_relative, and clear services along with setting the background_r, background_g, background_b params.
### Swim to Goal
This script allows you to input a coordinate as the goal, which the turtle then must reach while having a linear and angular velocity proportional to the remaining error. This builds off the previous scripts but also requires a proportional control drive, which will be expanded on in the following package.

Additionally, a launch file was created for each of these scripts to make it easier to run.


## PID Wall Following
My first goal with the real F1Tenth car was to create and implement a wall following algorithm using Proportional-Integral-Derivative, or PID, control. The scripts from this assignment can be found in the race package. The launch file is wall_follow.launch.

While working on this I learned the following:
- Connecting to, uploading code to, and running code on the computer on the F1Tenth car
- Accessing some data from the LIDAR sensor to calculate error
- Implementing the PID controller
- Tuning the PID coefficients
- Implementing a dynamic speed control based on error

This algorithm consisted of 2 nodes, dist_finder and control.

### Perception 
The perception node, dist_finder, used 2 points from the lidar scan. One point perpendicular to the path of the car and one 70 degrees ahead of that point, both on the right wall. Using these two values and a forwards projection, the error is calculated and published.

### Control
The control node subscribes to the error topic and uses the error to do a PID calculation. The proportional control is done by multiplying the error by a coefficient. The derivative control takes the previous error and finds the difference, multiplying it by a constant. The dynamic speed control takes the steering value, which is determined by the PID control, and slows down in a linear manner. The control node allows inputted PID values to make testing easier. The values that worked best were Kp = 15, Kd = 0.3, and Ki = 0.

## Demo
https://github.com/user-attachments/assets/95991caf-d580-4cd8-85aa-77dc1064222a


## Follow the Gap
While the PID wall follower worked on a simple track with no obstacles, it failed on anything more complex. The PID Wall Follower only used 2 points of data from the LIDAR sensor. Instead of ignoring the rest of the information, it can be used to create a algorithm able to react to any obstacles.

While working on this I learned the following:
- Interpreting data from the LIDAR sensor to analyze surroundings
- Using Rviz to visualize data from LIDAR and visualize the chosen path
- Implementing a reactive follow the gap algorithm
- Implementing a dynamic speed control based on error and distance to obstacle

This algorithm consists of 2 nodes, gap_finder and ftg_control.

## Perception and Planning
The perception and planning occurs in the gap_finder node. This node subscribes to the scan topic and uses the LIDAR data to find any edges by checking for jumps in depth. These edges are then extended according to the distance from the car to make up for the size of the car. 

A gap is chosen by finding the widest gap with a certain depth. If no gap meets this depth the process is repeated with a lower requirement for a gap. The angle error for this gap is published to the error channel while a marker message used to visualize with a green arrow is published to the visualization_marker channel.

## Control
The control node subscribes to the error channel and proportionally scales the error to the steering angle. Additionally, the dynamic speed control takes into account both the steering angle, which is based on error, and the distance to the obstacle infront of the vehicle. 

## Demo
https://github.com/user-attachments/assets/dd0f40eb-22e4-4f62-926d-d6683d0defa7

