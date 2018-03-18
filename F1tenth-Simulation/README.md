# F1tenth - PC Setup for Gazebo Simulation

The F1tenth platform can be simulated using Gazebo. 

## Install Dependencies

* Once all the required installations have been performed, install the following:

serial:

``` 
sudo apt-get install ros-<distro>-serial
```
controller_manager:

``` 
sudo apt-get install ros-<distro>-controller-manager
```
gazebo_ros_control:

``` 
sudo apt-get install ros-<distro>-gazebo-ros-control
```
joint_state_controller:

``` 
sudo apt-get install ros-<distro>-joint-state-controller 
```
effort_controllers

``` 
sudo apt-get install ros-<distro>-effort-controllers
```

* Replace SLAM launch files: -

``` 
roscd hector_slam_launch/launch/
```

Replace the tutorial.launch file with the one [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Simulation/launch_files).

If you want to replay a ros_bag or use the gazebo simulator, change the parameter "/use_sim_time" to True in tutorial.launch.

```
roscd hector_mapping/launch
```

Replace the mapping_default.launch file with the one [here](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Simulation/launch_files).

## Build from sources 
``` 
cd ~/racecar && catkin_make
```

## Launch the simulator 
``` 
source ~/catkin_ws/devel/setup.bash 
```
``` 
roslaunch racecar_gazebo racecar.launch
```
	 
## Virtual Field Test

Perform the steps in [F1tenth-Field](https://github.com/BU-DEPEND-Lab/Racecar/tree/master/F1tenth-Field)
