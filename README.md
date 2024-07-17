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
My first goal with the real F1Tenth car was to create and implement a wall following algorithm using Proportional-Integral-Derivative, or PID, control. The scripts from this assignment can be found in the race package.

While working on this I learned the following:
- Connecting to, uploading code to, and running code on the computer on the F1Tenth car
- Interpreting the data from the LIDAR sensor to calculate error
- Implementing the PID controller
- Tuning the PID coefficients
- Implementing a dynamic speed control based on error

This algorithm consisted of 2 nodes, dist_finder and control.

### Perception 
The perception node, dist_finder, used 2 points from the lidar scan. One point perpendicular to the path of the car and one 70 degrees ahead of that point. Using these two values and a forwards projection, the error is calculated and published.

### Control
The control node subscribes to the error topic and uses the error to do a PID calculation. The proportional control is done by multiplying the error by a coefficient. The derivative control takes the previous error and finds the difference, multiplying it by a constant. The dynamic speed control takes the steering value, which is determined by the PID control, and slows down in a linear manner. The control node allows inputted PID values to make testing easier. The values that worked best were Kp = 15, Kd = 0.3, and Ki = 0.

## Follow the Gap
