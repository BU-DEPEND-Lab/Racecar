# Racecar

This is the repo for the BU F1tenth team - http://sites.bu.edu/depend/bu-f110/

Demo video - https://www.youtube.com/watch?v=kTw-X1VolPk

## Installations

To perform field tests on the physical platform, perform the following installation steps 1 - 4 on the NVIDIA Jetson TK1 or TX1 and then continue [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Field).

To perform tests in the robotics simulator 'Gazebo', perform the following installation steps 1 - 4 on a PC and then continue [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Simulation).

The code has been tested on:
* Ubuntu 16.04, Ros Kinetic, and Gazebo 7, for the TX1
* Ubuntu 14.04, Ros Indigo, and Gazebo 2, for the TK1

### 1) Clone repo

Start by cloning the GitHub repo. The required files can also be transferred to the TK1, via an SD card.


### 2) ROS installation and sensor integration

Follow the instructions provided by the F1tenth website - http://f1tenth.org/lectures

Replace SLAM launch files: -

``` 
roscd hector_slam_launch/launch/
```

For Field Tests, replace the tutorial.launch file with the one [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Field/launch_files).

For simulations, replace the tutorial.launch file with the one [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Simulation/launch_files).

```
roscd hector_mapping/launch
```

For Field Tests, replace the mapping_default.launch file with the one [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Field/launch_files).

For simulations, replace the tutorial.launch file with the one [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Simulation/launch_files).


### 3) Install Radler

Follow the steps provided in https://sri-csl.github.io/radler/

### 4) Create a new ROS package

Add all the src files from Racecar/Robot to a new package. The provided instructions below assume the package to be named 'race'.

## BRASS demo (Deprecated)
Since there isn't GPS in the F1_tenth system, we have to modified the ekf node in order to make it work with our system. The modified code is in the folder modified_from_brass.
 
Copy ekf.py and replace the one in the folder BRASS/models/brass/src/navigation_phase1/phase1A_v2. 

Copy adapter.py and replace the one in BRASS/models/brass/src/controller/monitor

All the launch file used in mit demo are in zed-ros-wrapper/launch folder. Modified ekf and adapter are in modified_from_brass folder. race/src/ folder adds a drive straight for 5 seconds node and a fakeGps node.

## Adaptive control using Caffe (Deprecated)
zed-ros-wrapper folder is for driving the zed camera in ros. Just simply put it in a catkin workspace and make it.
