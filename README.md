# RBE501

## Background

This project was designed for the RBE501 course offered at Worcester Polytechnic Institute. Inside contains contents to control the Kortex Kinova Gen3 arm using teleop through a position based controller. This code was tested using the 7dof configuration. 6dof will require the screw axes to be adjusted accordingly.

## How to Use
There are three types of controllers used in this package; moveit, trajectory, and pure position based controllers. Each works through a teleop through keyboard inputs. The teleop works through determining the end effector position from selecting either a positional update (translating across the xyz plane) or with orientation (roll pitch yaw).

To move positionally, select either i, j, or k to determine an axis to translate across. Then, select either t or b to determine a positive or negative multiplier. Pressing q or e will move the arm in the selected direction by either 10 or 1 cm, respecively. The default resolution is 10 cm.
To move with a selected roll pitch or yaw, press r, p, or y. The default angle will be 1 degree. Select w or x to change the resolution to 1 degree of 15 degrees, respectively. Additionally, the t/b multiplier also applies here.

## MoveIt Based Control
Please use a different terminal for each command

$ roslaunch kortex_gazebo spawn_kortex_robot.launch
$ roslaunch ros_control moveItServer.launch
$ rosrun ros_control moveItClient.py

Notably, this code will initalize the robot EE to be not in the home configuration. 

## Trajectory & Position Based Control

Please use a different terminal for each command

$ roslaunch kortex_gazebo spawn_kortex_robot.launch
$ rosrun ros_control jointTrajectoryClient.py
$ rosrun ros_control ik_solver.py

Please note, the jointTrajectoryClient.py was misnamed and actually serves as a service. The ik_solver.py is the true client. 

The user input on the terminal will allow the user to select for trajectory or position based control as well as the type of jacobian used. The analytical jacobian will not calculate orientation while the space jacobian will. So, if the teleop mode requires a specific orientation, select the space jacobian.

