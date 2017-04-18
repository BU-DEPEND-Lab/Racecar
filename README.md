# F1tenth

## F1tenth competition

## BRASS demo
Since there isn't GPS in the F1_tenth system, we have to modified the ekf node in order to make it work with our system. The modified code is in the folder modified_from_brass.
 
Copy ekf.py and replace the one in the folder BRASS/models/brass/src/navigation_phase1/phase1A_v2. 

Copy adapter.py and replace the one in BRASS/models/brass/src/controller/monitor

All the launch file used in mit demo are in zed-ros-wrapper/launch folder. Modified ekf and adapter are in modified_from_brass folder. race/src/ folder adds a drive straight for 5 seconds node and a fakeGps node.

## Adaptive control using Caffe
zed-ros-wrapper folder is for driving the zed camera in ros. Just simply put it in a catkin workspace and make it.
## Using Slam with lidar:
This is for sending the raw data from lidar to the hector_slam node and generate map for surrounding enviroment. If you want to replay a ros_bag, please change the parameter "/use_sim_time" to Ture in tutorial.launch.

http://f1tenth.org/lab_instructions/W3_T1_Using%20the%20Hector%20SLAM.pdf

Please following the above link to set Slam up.Then first use the following command to go to the directory:

roscd hector_slam_launch/launch/

replace the tutorial.launch with the one in the Slam_launch folder.

Then use the command to another directory:

roscd hector_mapping/launch

repalce the mapping_defualt.launch with the one in the Slam_launch folder.

then simply run the the launch file using the command:
roslaunch hector_slam_launch tutorial.launch

this launch file will launch all nodes we are using in f1_tenth
