# Robot setup

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