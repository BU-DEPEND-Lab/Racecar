## ﻿Installation guide for mit-racecar simulator
## Tested on Ubuntu 16.04 & ROS Kinetic

## Download source codes ##
$ mkdir ~/racecar && cd ~/racecar
$ mkdir src && cd src
$ git clone https://github.com/mit-racecar/racecar.git
$ git clone https://github.com/mit-racecar/vesc.git
$ git clone https://github.com/mit-racecar/racecar-simulator.git

## Install Dependencies ##
ackermann_msgs:
	$ sudo apt-get install ros-<distro>-ackermann-msgs
serial:
	$ sudo apt-get install ros-<distro>-serial
controller_manager:
	$ sudo apt-get install ros-<distro>-controller-manager
gazebo_ros_control:
	$ sudo apt-get install ros-<distro>-gazebo-ros-control
joint_state_controller:
	$ sudo apt-get install ros-<distro>-joint-state-controller
effort_controllers
	$ sudo apt-get install ros-<distro>-effort-controllers


## Build from sources ##
$ cd ~/racecar && catkin_make


## Modify keyboard_teleop.py ##
$ cd ~/racecar/src/racecar-simulator/racecar_control/scripts
open keyboard_teleop.py:

	change line 40 
	"pub = rospy.Publisher('/racecar/ackermann_cmd_mux/input/teleop', 				AckermannDriveStamped)" 
	to:
	"pub = rospy.Publisher('/vesc/high_level/ackermann_cmd_mux/input/default', 			AckermannDriveStamped, queue_size=1)"


## Launch the simulator ##
$ source ~/racecar/devel/setup.bash
$ roslaunch racecar_gazebo racecar_tunnel.launch
or
$ roslaunch racecar_gazebo racecar.launch


## How to control ##
A keyboard controller will pop-up
Click the controller, control the car using W-A-S-D
