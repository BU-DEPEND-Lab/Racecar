# VIRTUAL ROADTEST
A Race track generating tool that uses a genetic algorithm-based approach to test the performance of steering controllers deployed on simulation models of DEPEND lab's F1/10th platform


## Usefule commands
``` /home/f1/radler/radler.sh --ws_dir /home/f1/catkin_ws/src compile pid_controller.radl --ROS ```


* To compile the planner:

``` g++ -std=c++0x -ggdb `pkg-config --cflags opencv` -o `basename Planning_Gazebo_Customized_GA_version.cpp .cpp` Planning_Gazebo_Customized_GA_version.cpp `pkg-config --libs opencv` ```


* To compile the pid controller:
	``` catkin_make ```


* modify pid_controller.cpp under:
``` ~/catkin_ws/src/race/src/radler/pure_pursuit/src/pid_controller.cpp ``` 

* the 'main' function for the radler system is under:
``` ~/catkin_ws/src/ros/pid_controller/src/radl__pid_controller.cpp ```
