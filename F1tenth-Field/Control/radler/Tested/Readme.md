Old Notes that may be useful:


$ ./radler.sh --ws_dir {catkin_ws}/src compile {path/to/.radl/file} --ROS

example: $ ./radler.sh --ws_dir /tmp/catkin_ws/src compile /home/bu/race_ws/Racecar/radler/pure_pursuit/pid_controller.radl --ROS


Note: Two paths need to change:
 stopsign_detection(if cam_gpu version) line27: /path/to/stopsign_classifier.xml
 PidController.cpp   in PidController::PidController (): /path/to/path.txt\
 
the code might not be fully funcional.
