# Racecar

This is the repo for the BU F1tenth team - http://sites.bu.edu/depend/bu-f110/

Demo video - https://www.youtube.com/watch?v=kTw-X1VolPk

## Installations

### 1) Clone repo

Start by cloning the GitHub repo. The required files can also be transferred to the TK1, via an SD card.

### 2) ROS installation and sensor integration

Follow the instructions provided by the F1tenth website - http://f1tenth.org/lectures

Next follow the following steps: -

	$ roscd hector_slam_launch/launch/

Replace the tutorial.launch file with the one in Racecar/Robot/Custom_Files.

If you want to replay a ros_bag or use the gazebo simulator, please change the parameter "/use_sim_time" to True in tutorial.launch.

	$ roscd hector_mapping/launch

Replace the mapping_defualt.launch file with the one in Racecar/Robot/Custom_Files.

### 3) Install Radler

Follow the steps provided in https://sri-csl.github.io/radler/

### 4) Create a new ROS package

Add all the src files from Racecar/Robot to a new package. The provided instructions below assume the package to be named 'race'.

## Robot setup

Once all the required installations have been performed, run the following commands to prepare the robot for autonomous control.

### 1) Start the SLAM nodes:
      $ roslaunch hector_slam_launch tutorial.launch

The current setup does not consider a prebuilt SLAM configuration space (2D map of surroundings). Therefore this has to be built manually, which can be done in 2 ways:

### a) Manual control using keyboard:
      $ rosrun race keyboard.py 

### b) Manual control using xbox controller:
      $ rosrun race joystick.py 

Choose one of the above methods and manually maneuver the car in an environment (room, corridor etc.) to build the map. Upon completion, Stop(CTRL+C) the chosen manual control node.

### 2) Launch the AHS path planner:

With a built map in hand, execute the path planner. When prompted, give the desired coordinates you want the robot to move to. The initial starting coordinate is 1024,1024.

      $ cd ~/catkin_ws/src/race/src
      $ ./AHS

Once a path is found, this will be indicated on the display.

### 3) Start the Radler gateway:
      $ cd ~/catkin_ws/devel/lib/pid_controller
      $ ./controller_gateway

### 4) Launch the pid controller:
      $ cd ~/catkin_ws/devel/lib/pid_controller
      $ ./pid_controller

The robot should now autonomously maneuver to the desired location.

## BRASS demo (Deprecated)
Since there isn't GPS in the F1_tenth system, we have to modified the ekf node in order to make it work with our system. The modified code is in the folder modified_from_brass.
 
Copy ekf.py and replace the one in the folder BRASS/models/brass/src/navigation_phase1/phase1A_v2. 

Copy adapter.py and replace the one in BRASS/models/brass/src/controller/monitor

All the launch file used in mit demo are in zed-ros-wrapper/launch folder. Modified ekf and adapter are in modified_from_brass folder. race/src/ folder adds a drive straight for 5 seconds node and a fakeGps node.

## Adaptive control using Caffe (Deprecated)
zed-ros-wrapper folder is for driving the zed camera in ros. Just simply put it in a catkin workspace and make it.

# Simulation

## Install Dependencies
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

## Build from sources 
	$ cd ~/racecar && catkin_make

## Launch the simulator 
	$ source ~/catkin_ws/devel/setup.bash
	$ roslaunch racecar_gazebo racecar.launch
