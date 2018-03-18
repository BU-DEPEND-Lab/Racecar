# Racecar

This is the repo for the BU F1tenth team - http://sites.bu.edu/depend/bu-f110/

Demo video - https://www.youtube.com/watch?v=kTw-X1VolPk

## Installations

### 1) Clone repo

Start by cloning the GitHub repo. The required files can also be transferred to the TK1, via an SD card.

### 2) ROS installation and sensor integration

Follow the instructions provided by the F1tenth website - http://f1tenth.org/lectures

Next follow the following steps: -

``` 
roscd hector_slam_launch/launch/
```

Replace the tutorial.launch file with the one in Racecar/Robot/Custom_Files.

If you want to replay a ros_bag or use the gazebo simulator, please change the parameter "/use_sim_time" to True in tutorial.launch.

```
roscd hector_mapping/launch
```

Replace the mapping_defualt.launch file with the one in Racecar/Robot/Custom_Files.

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
