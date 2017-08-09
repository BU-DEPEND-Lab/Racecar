#!/usr/bin/env python

import rospy
from race.msg import drive_param
from race.msg import pid_input
#import curses	# kill


servo_offset = 18.5
prev_error = 0.0
kp = 14.0
kd = 0.09
#vel_input = 5.0
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
#stdscr = curses.initscr() # kill

#key = '' # kill
#key = stdscr.getch() # kill
#if key == curses.KEY_DC:    # kill
#	angle = 0
#	vel_input = 0
	
def control(data):
	global prev_error
	global vel_input
	global kp 
	global kd

	## Your code goes here
	# 1. Scale the error
	# 2. Apply the PID equation on error
	# 3. Make sure the error is within bounds
	error = data.pid_error * 10
	angle = -kp * error - kd *(error - prev_error)
	angleRad = 180 / angle
	if angleRad < -0.78:
		angleRad = -0.78
	elif angleRad > 0.78:
		angleRad = 0.78 
	prev_error = error 	


	## END

	msg = drive_param();
	msg.velocity = data.pid_vel	
	msg.angle = angleRad
	print(angle)
	print(data.pid_vel)
	pub.publish(msg)
	#curses.endwin()
	

if __name__ == '__main__':
	'''global kp
	global kd
	print("Listening to error for PID")
	kp = input("Enter Kp Value: ")
	kd = input("Enter Kd Value: ")
	#vel_input = input("Enter Velocity: ")'''
	rospy.init_node('pid_controller', anonymous=True)
	rospy.Subscriber("error", pid_input, control)
	rospy.spin()
