#####################################################
# This program automates the testing process for the adaptive testing environment on the f1/10 robot
# - Takes number of iterations(loops) as an argument (e.g to run with 5 loops, run: "python master_test_script.py 5" )
#
# 1) run java map world generator - o/p ==> 2D array(.png file) and .world file
# 2) run roslaunch files. These should terminate once goal is reached
#		o/p ==> .csv files , path coordinates , completion time
# 3) run AI to adjust i/p parameters to java map world generator
# 4) back to step 1. (Define number of loops)
# 
# TODO:  - see TODOs thoughout code below
#####################################################

#!/usr/bin/env python
import os
import subprocess
import sys
import time
import threading
import signal
import logging
from setproctitle import *

setproctitle('master_test') # set process name for ROS to identify

TIMEALLOWED = 10 # change to a large number(e.g. 5 minutes) for actual tests

""" Clear all existing generated files"""
""" TODO: implement the Initialize function once the AI code is made"""
def Initialize():
	#delete bag files
	os.chdir('/home/f1/catkin_ws/src/racecar_simulator/racecar_gazebo/launch/test_runs/test_results')
	subprocess.call('rm *.bag', shell=True)	

""" runs roslaunch file to implement test run in gazebo and terminates once the robot reaches its goal"""


def ROS():
	try:
		cmd1 = 'roslaunch racecar_gazebo racecar_tunnel.launch'
		simulation1 = subprocess.Popen(cmd1, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 
		ros_node_pid1 = os.getpgid(simulation1.pid)

		time.sleep(3)

		cmd2 = 'roslaunch hector_slam_launch tutorial.launch'
		simulation2 = subprocess.Popen(cmd2, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 
		ros_node_pid2 = os.getpgid(simulation2.pid)

		#kill after allowed time is exceeded
		time.sleep(TIMEALLOWED)
		logging.info('Timout - Goal not reached within 5 minutes...')

		subprocess.call('killall gzserver', shell=True)

		time.sleep(2)
		os.killpg(ros_node_pid1, signal.SIGTERM)  # Send the signal to all the process groups
		time.sleep(2)
		os.killpg(ros_node_pid2, signal.SIGTERM)  # Send the signal to all the process groups
	except KeyboardInterrupt:
		print "KILL CALLED"
		subprocess.call('killall gzserver', shell=True)

		time.sleep(2)
		os.killpg(ros_node_pid1, signal.SIGTERM)  # Send the signal to all the process groups
		time.sleep(2)
		os.killpg(ros_node_pid2, signal.SIGTERM)  # Send the signal to all the process groups
		print "KILLED ALL"

	logging.info('ROS ending...')


""" Output is .png file (2D array of path) and .world file."""
# TODO: - standardize location of .jar file and outputs 
def WorldGenerator():
	os.chdir('/home/f1/f1tenth/gzbo2_generator/')
	subprocess.call('java -jar Generator.jar', shell=True)
	logging.info('WorldGenerator ending...')


def AI(csv_file, completion_time):
	subprocess.call('run AI code')


"""TODO: Implement the Latest _file function based on the requirements of the AI code Eddie writes """
""" add a bag to .csv convertion function. You (Akash) can use the one that was sent via email."""
""" Function to find the last modified (newest) file with a given file extension (e.g '.bag') and 
	path to the concerned directory. Both parameters should be strings"""
def Latest_file(file_extension, path):
	dated_files = [(os.path.getmtime(fn), os.path.basename(fn)) 
               for fn in os.listdir(path) if fn.lower().endswith(file_extenstion)]
	dated_files.sort()
	dated_files.reverse()
	newest = dated_files[0][1]
	return newest

def main():

	logging.basicConfig(filename='/home/f1/f1tenth/test.log', level=logging.DEBUG,format='%(asctime)s:%(levelname)s:%(message)s')


	''' number of iterations'''
	if(len(sys.argv) == 2):
		try:
			iterations = int(sys.argv[1])
		except ValueError:
			print('Input is not an integer')
			sys.exit()
	else:
		print('Provide number of iterations')
		sys.exit()


	''' main loop'''
	
	for i in range(0,iterations):
		'''STEP 1'''
		#WorldGenerator()
		#time.sleep(2)
		#logging.info('WorldGenerator ended')

		'''STEP 2'''
		ROS()
		logging.info('ROS ended')
		'''STEP 3'''
		#AI()
		time.sleep(2)
	
	print('program ended')

if __name__ == "__main__":
   main()