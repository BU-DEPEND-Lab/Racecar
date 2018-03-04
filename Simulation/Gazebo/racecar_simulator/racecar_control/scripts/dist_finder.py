#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input
import numpy as np

desired_trajectory = 1

pub = rospy.Publisher('error', pid_input, queue_size=10)

##	Input: 	data: Lidar scan data
##			theta: The angle at which the distance is requried
##	OUTPUT: distance of scan at angle theta
def getRange(data,theta):
# Find the index of the arary that corresponds to angle theta.
# Return the lidar scan value at that index
# Do some error checking for NaN and ubsurd values
## Your code goes here
	ranges = data.ranges
	index = int ( (theta + 45) / 0.25)	
	return ranges[index]

def callback(data):
	theta = 50;
	#omega = 90;
	a = getRange(data,theta)
	b = getRange(data,0)

	swing = math.radians(theta)
	print(b)
	## Your code goes here
	#tangle = (a * np.cos(theta) - b) / a * np.sin(theta)
	#alpha = np.arctan(tangle)
	#AB = b * np.cos(alpha)
	#AC = 10 # 0.5
	#CD = AB + AC * np.sin(alpha)
	#error = CD - 1

	if (b < 1):
		error = -0.4
	if (b > 1):
		error = 0.4
	vel = 8
	#frontDist = getRange(data, omega)
	#if frontDist <= 1:
	#	vel = 8
	#else:
	#	vel = 8
## END

	msg = pid_input()
	msg.pid_error = error
	msg.pid_vel = vel
	pub.publish(msg)
	

if __name__ == '__main__':
	print("Laser node started")
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("scan",LaserScan,callback)
	rospy.spin()
