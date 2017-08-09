This program (master_test_script.py) automates the testing process for the adaptive testing environment on the f1/10 robot.



To execute:

$ source ~/catkin_ws/devel/setup.bash
$ roscd racecar_gazebo/launch/test_runs
$ python master_test_script.py <number of iterations>  (e.g. 'python master_test_script.py 2' ....for 2 iterations)


TODO:
1- links to folder structure are hardcoded. This needs to be modified to run on different users / workstations see:

	-racecar_result.launch
	-gazebo_test_run_NO_EKF.launch
	-master_test_script.py

2- the hector_slam node is getting odom from gazebo. This needs to be changed to reflect the robots actual odom source, which comes from hector_slam

3- rate of robot_state_publisher needs to be reduced. This may be causing the poor performance of hector_slam on gazebo. (real life demos work fine)

4- the SIGINT signal to master_test_script.py is sent from the pure_pursuit_controller_gazebo_testing_version.py in the 
   'racecar_gazebo' package, NOT the 'race' package.

5- a few more TODOs are listed in the files mentioned in '1'



DEPENDENCIES:

install setptoctitle:
$ pip install setproctitle

install psutil:
$ sudo apt-get install gcc python-dev python-pip
$ pip install psutil

note: Custom messages from the 'race' package are used.
