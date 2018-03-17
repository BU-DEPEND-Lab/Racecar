#!/usr/bin/env python

import rospy
from race.msg import odometry
from geometry_msgs.msg import PoseStamped
import sys
import serial

pub = rospy.Publisher('slam_odometry', odometry, queue_size=10)
# callback function on occurance of drive parameters(angle & velocity)
def callback(data):
	global velocity_prev
        global x_position_prev
        global y_position_prev
        global z_position_prev
	global prev_sec
	global prev_nsec
	sec = data.header.stamp.secs
	nsec = float(data.header.stamp.nsecs)
	time = (sec - prev_sec) + (nsec - prev_nsec) / 10**9
	x_position = data.pose.position.x
	y_position = data.pose.position.y
	z_position = data.pose.position.z
	x_velocity = (x_position - x_position_prev) / time
	y_velocity = (y_position - y_position_prev) / time
	z_velocity = (z_position - z_position_prev) / time
	x_acce = (x_velocity - velocity_prev[0]) / time
	y_acce = (y_velocity - velocity_prev[1]) / time
	z_acce = (z_velocity - velocity_prev[2]) / time
	velocity_list = [x_velocity,y_velocity,z_velocity]
	acceleration_list = [x_acce,y_acce,z_acce]
	msg = odometry()
	msg.speed = velocity_list
	msg.alpha = acceleration_list
	pub.publish(msg)
	prev_sec = sec
	prev_nsec = nsec
	velocity_prev = velocity_list
	x_position_prev = x_position
	y_position_prev = y_position
	z_position_prev = z_position

def starter():
	rospy.init_node('odometry', anonymous=True)
	global x_position_prev
        global y_position_prev
        global z_position_prev
	global prev_sec
	global prev_nsec
	prev = PoseStamped()
	x_position_prev = prev.pose.position.x
	y_position_prev = prev.pose.position.y
	z_position_prev = prev.pose.position.z
	prev_sec = prev.header.stamp.secs
	prev_nsec = prev.header.stamp.nsecs
	rospy.Subscriber("slam_out_pose",PoseStamped, callback)	
	rospy.spin()

if __name__ == '__main__':
	global velocity_prev
	velocity_prev = [0.0,0.0,0.0]
	print("odometry calculating initialized")
	starter()
