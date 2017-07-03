#!/usr/bin/env python

# PURE PURSUIT CONTROLLER


#python includes
import numpy as np 
import math
import time
import rospy
import tf

#ROS messages
from racecar_control.msg import drive_param
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

#published topics

#em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
pub= rospy.Publisher('drive_parameters', drive_param, queue_size=1)

#global variables
global look_ahead_dist 
global goalRadius
global prev_error
global velocity
global kp 
global kd
global flag
global planner_coord
global path_y
global path_x
global look_ahead_dist

prev_error = 0
flag = 0
kp = 1 #14.0
kd = 0.09

look_ahead_dist = 1.0  # 1m look ahead distance


'''create robot and initialize parameters'''
class robot:
	def __init__(self):
		#steer control
		self.x = 0.0
		self.y = 0.0
		self.yaw = 0.0
		#speed control
		self.speed = 0.0

	'''set robot pose and orientation'''
	def setPose(self, new_x, new_y, new_yaw):
		self.x = new_x
		self.y = new_y
		self.yaw = new_yaw

	def accelerate(self, rate):
		self.speed += rate

	def deccelerate(self, rate):
		self.speed -= rate

'''read incoming data from path planner'''
def desired_track(data):
	global flag
	global path_y
	global path_x
	global planner_coord

	planner_coord = np.size(data.data) / 2 #number of coords
		
	p = np.reshape(data.data,((planner_coord),2))

	path_y = [0.0 for ii in range (planner_coord)]
	path_x = [0.0 for ii in range (planner_coord)]
	
	for i in range(0,planner_coord):
		path_y[i] = p[i][1]
		path_x[i] = p[i][0]
	flag = 1

def poseUpdate(data):
	#orientation data
	quaternion = (
		data.pose.orientation.x,
		data.pose.orientation.y,
		data.pose.orientation.z,
		data.pose.orientation.w)
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

	#pose data
	x = data.pose.position.x
	y = data.pose.position.y
	return x, y, yaw


def steer_err(carrot_x, carrot_y, curr_x, curr_y , curr_yaw):

	delta_x = carrot_x - curr_x
	# l = look ahead distance from robot to carrot
	l = np.sqrt((carrot_x - curr_x)**2 + (carrot_y - curr_y)**2)
	print ("l = ", l)


	theta = (np.arcsin(delta_x / l)) - 1.56 # steering angle required without taking current orientation into consideration
	#theta = (np.arctan((carrot_x - curr_x)/(carrot_y - curr_y)))

	print ("theta = ", theta)
	print ("curr_yaw = ", curr_yaw)

	'''
	if curr_yaw >= 0.0:
		steer_error = theta + curr_yaw
	elif curr_yaw < 0.0:
		steer_error = theta - curr_yaw
	'''
	steer_error =  curr_yaw - theta
	return steer_error


def PD_controller(steer_error, diff_error, kp, kd):
	steerAngle = - (kp * steer_error) - (kd * diff_error)
	#steerAngle = 0.9 * steer_prev + 0.1 * steerAngle
	#steer_prev = steerAngle

	if steerAngle < -0.4 :
		steerAngle = -0.4
	elif steerAngle > 0.4 :
		steerAngle = 0.4  

	return steerAngle

def goalCheck(goal_x, goal_y, curr_x, curr_y):
	goalRadius = np.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
	if goalRadius < 0.5 : 
		return True
	else :
		return False

def control(data):    
	msg = drive_param(); 

	global prev_error
	global velocity
	global kp 
	global kd
	global flag
	global planner_coord
	global path_y
	global path_x
	global look_ahead_dist

	myrobot = robot() 

	''' flag is set when path arrives from planner node'''    
	while flag == 1:        
		####### update pose
		curr_x, curr_y, yaw = poseUpdate(data)   
		myrobot.setPose(curr_x, curr_y, yaw)


		goal_x = path_x[planner_coord-1]
		goal_y = path_y[planner_coord-1]

		if goalCheck(goal_x, goal_y, myrobot.x, myrobot.y) == True :
			flag = 0
			angle = 0.0
			speed = 0.0
			print ("GOAL REACHED")
			pub.publish(msg) 
			break

		######## find carrot point on path
		possible_l = [0]*planner_coord
		for i in range(0, planner_coord):
				possible_l[i] = abs((path_x[i] - myrobot.x)**2 + (path_y[i] - myrobot.y)**2 - look_ahead_dist)

		j = possible_l.index(min(possible_l))

		carrot_x = path_x[j]
		carrot_y = path_y[j]
		print ("carrot_x = ", carrot_x, "carrot_y = ", carrot_y)

		######### steer_ error is the angle error value
		steer_error = steer_err(carrot_x, carrot_y, myrobot.x, myrobot.y , myrobot.yaw)
		print ("steer_error = ", steer_error)

		######## calculate output steering angle using pd controller'''
		#diff_error = steer_error - prev_error    # differential
		#steerAngle = PD_controller(steer_error, diff_error, kp, kd)
		#prev_error = steerAngle

		steerAngle = -steer_error
		
		msg.angle = steerAngle
		speed = 4.0  # REMOVE THIS!!
		msg.velocity = speed
		print("flag = 1, vel = " , msg.velocity, ", angle = " , msg.angle)
		pub.publish(msg) 
		flag = 0
	
	curr_x, curr_y, yaw = poseUpdate(data)   
	myrobot.setPose(curr_x, curr_y, yaw)
	''' keep vel and steering to 0 if no plan arrives and flag is not set'''
	angle = 0.0
	speed = 0.0
	msg.angle = angle
	msg.velocity = speed  # constant velocity
	#print("flag = 0, vel = ", msg.velocity, ", angle = " , msg.angle)
	#pub.publish(msg) 
	#rospy.Rate(10)

if __name__ == "__main__":
	rospy.init_node("pid_controller", anonymous=True)
	rospy.Subscriber("path_planner", Float64MultiArray, desired_track) # from path planner
	rospy.Subscriber("slam_out_pose", PoseStamped, control) # from slam

	rospy.spin() 



'''
def velocityControl():  # open loop control
	max_speed = 40
	max_speex_reverse = -40
	min_speed = 0

	if (steet_error_is_increasing_too_much):
		while(myrobot.speed > 20 ):
			myrobot.deccelerate(5)

	else:
		while(myrobot.speed < 35 ):
			myrobot.accelerate(5)

def emergencyStop():
	t_end = time.time() + 5
	while time.time() < t_end:
		#do whatever you want to do
	em_pub.publish(True) # find a way to manually chage to false
'''