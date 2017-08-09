#!/usr/bin/env python

# PURE PURSUIT CONTROLLER for steering angle
# PD controller for speed control


# python includes
import numpy as np
import math
import time
import rospy
import tf
import subprocess
import signal
import os
import psutil

# ROS messages
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

# published topics

#em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
pub = rospy.Publisher('drive_parameters', drive_param)

#global variables
global look_ahead_dist
global goalRadius
global prev_steer
global prev_speed
global speed
global flag
global planner_coord
global path_y
global path_x
global speeds
global look_ahead_dist
global myrobot

prev_steer = 0.
prev_speed = 0.
flag = 0


'''create robot and initialize parameters'''


class robot:
    def __init__(self):

        # steer control variables:
        self.x = 0.
        self.y = 0.
        self.yaw = 0.

        # speed control variables:
        self.speed = 0.

    '''set robot pose and orientation'''

    def setPose(self, new_x, new_y, new_yaw):
        self.x = new_x
        self.y = new_y
        self.yaw = new_yaw

    def setSpeed(self, new_speed):
        self.speed = new_speed

"""

Read incoming data from path planner

	>>> data is a 1D array of x position, y position, speed, x position, y position , speed ... and so on.

	>>> note that speed is a percentage of max speed that is required

"""
myrobot = robot()

def desired_track(data):
    global flag
    global path_y
    global path_x
    global speeds
    global planner_coord

    planner_coord = np.size(data.data) / 3  # number of coords

    p = np.reshape(data.data, ((planner_coord), 3))

    path_y = [0. for ii in range(planner_coord)]
    path_x = [0. for ii in range(planner_coord)]
    speeds = [0. for ii in range(planner_coord)]

    for i in range(0, planner_coord):
        path_x[i] = p[i][0]
        path_y[i] = p[i][1]
        speeds[i] = p[i][2]
    flag = 1


""" Update pose data from hector_slam """

def dist(x1, x2, y1, y2):
    d = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    return round(d, 2)


def poseUpdate(data):
    # orientation data
    quaternion = (
        data.pose.orientation.x,
        data.pose.orientation.y,
        data.pose.orientation.z,
        data.pose.orientation.w)
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

    # pose data
    x = data.pose.position.x
    y = data.pose.position.y
    return x, y, yaw


""" Set Look ahead distance """

def look_ahead(curr_speed):
    
    if curr_speed < 2 :
        look_ahead_dist = 1
    if curr_speed > 2 and curr_speed < 10 :
        look_ahead_dist = 1
    if curr_speed > 10 :
        look_ahead_dist = 1.5
    return look_ahead_dist
""" calculate desired steering angle """


def calculate_desired_steer(carrot_x, carrot_y, curr_x, curr_y, curr_yaw):

    delta_x = carrot_x - curr_x
    # l = look ahead distance from robot to carrot
    print curr_x , curr_y, carrot_x, carrot_y
    l = np.sqrt((carrot_x - curr_x)**2 + (carrot_y - curr_y)**2)
    #l = dist(carrot_x, curr_x, carrot_y, curr_y)
    print ("l = ", l)

    # subtract 1.56 because of rotated x, y axes....refer to report
    theta = -(np.arcsin(delta_x / l))+1.56
    #theta = +(np.arcsin(delta_x / l))-1.56
    print ("theta = ", theta)
    print ("curr_yaw = ", curr_yaw)

    desired_steer = curr_yaw - theta

    return desired_steer


""" PD controller for steering """


def PD_controller(desired_steer, diff_error):
    kp = 1
    kd = 0.09

    steerOutput =  -(kp * desired_steer) - (kd * diff_error)
    if((steerOutput < 0.1) and (steerOutput > 0.)):
	steerOutput = 0.1
    elif((steerOutput > -0.1) and (steerOutput < 0.)):
        steerOutput = -0.1
    elif steerOutput < -0.5:
        steerOutput = -0.5
    elif steerOutput > 0.5:
        steerOutput = 0.5

    return steerOutput


""" open loop controller for speed """


def speedControl(speed_percentage, prev_speed):
    max_speed = 1
    max_speed_reverse = -1
    stop = 0

    desired_speed = speed_percentage * max_speed

    speed = 0.9 * prev_speed + 0.1 * desired_speed

    if speed < -max_speed_reverse and speed < stop:
        speed = -max_speed_reverse
    elif speed > max_speed and speed > stop:
        speed = max_speed

    return speed


""" Stop the car if it is within an acceptable distance from the goal point """


def goalCheck(goal_x, goal_y, curr_x, curr_y):
    #goalRadius = np.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
    goalRadius = dist(goal_x, curr_x, goal_y, curr_y)
    print goalRadius
    if goalRadius < 2.0:  # goalRadius > look_ahead_distance. If a smaller gaolRadius is requried, a new condition needs to be added
        return True
    else:
        return False

def EndTestRun():
    PROCNAME = 'master_test'

    for proc in psutil.process_iter():
        if proc.name == PROCNAME:
            proc_id = proc.pid
            break
    os.kill(proc_id, signal.SIGINT)


""" main control function """


def control(data):
    msg = drive_param()

    global prev_steer
    global prev_speed
    global speed
    global flag
    global planner_coord
    global path_y
    global path_x
    global speeds
    global look_ahead_dist
    global myrobot


    """ Flag is set when path and speed values arrive from planner node"""
    # Update pose
    curr_x, curr_y, yaw = poseUpdate(data)
    myrobot.setPose(curr_x, curr_y, yaw)

    if flag == 1:

        goal_x = path_x[planner_coord - 1]
        goal_y = path_y[planner_coord - 1]

        if goalCheck(goal_x, goal_y, myrobot.x, myrobot.y):
            flag = 0
            msg.angle = 0.
            msg.velocity = -2.0
            
            print ("GOAL REACHED")
            pub.publish(msg)
            EndTestRun()
            exit(0)

        # Set look ahead distance ######################################
        look_ahead_dist = look_ahead(myrobot.speed)
        print "Look Ahead = ",look_ahead_dist,"myrobot.speed = ", myrobot.speed


        # find starting point along the path --> closest point on the path

        
        startingPoints = [0.] * planner_coord
        for i in range(0, planner_coord):  
            d = dist(path_x[i], myrobot.x, path_y[i], myrobot.y)
            startingPoints[i] = round(d, 2)
        #    print ("startingPoints", i, " = ", startingPoints[i])    

               
        closest_point = min(startingPoints)        
        startIndex = startingPoints.index(closest_point)
        print ("startIndex = ", startIndex)            


        # Find carrot point on path
        possible_l = [0.] * planner_coord
        for i in range(startIndex, planner_coord):
            d = dist(path_x[i], myrobot.x, path_y[i], myrobot.y)
            difference = abs(d-look_ahead_dist)
            possible_l[i] = round(difference, 2)

        l_carrot = (min(possible_l[startIndex:]))
        print ("l_carrot = ", l_carrot)    

        for i in range(0, planner_coord):
            if i < startIndex:
                continue
            elif i >= startIndex:
                if possible_l[i] == l_carrot:
                   carrotIndex = i 
                   break

        print("carrotIndex = ", carrotIndex)  

        carrot_x = path_x[carrotIndex]
        carrot_y = path_y[carrotIndex]
        print ("carrot_x = ", carrot_x, "carrot_y = ", carrot_y)

        # Find desired steer value
        desired_steer = calculate_desired_steer(
            carrot_x,
            carrot_y,
            myrobot.x,
            myrobot.y,
            -myrobot.yaw)  # TEST IF NEGATION IS NEEDED IN GAZEBO
        print ("desired_steer = ", desired_steer)

        # Calculate steering output to publish using pd controller
        diff_error = desired_steer - prev_steer

        steerOutput = PD_controller(desired_steer, diff_error)
        prev_steer = steerOutput

        # Speed control
        speed = speedControl(speeds[carrotIndex], prev_speed)
        prev_speed = speed
        myrobot.speed = speed

        # Publish message
        msg.angle = steerOutput
        msg.velocity = speed
        print("flag = 1, vel = ", msg.velocity, ", angle = ", msg.angle)
        pub.publish(msg)
  
    # if flag = 0
    angle = 0.
    speed = 0.
    msg.angle = angle
    msg.velocity = speed
    #pub.publish(msg)
    # rospy.Rate(10)

def myhook():
    msg = drive_param()
    angle = 0.
    speed = -1.0
    msg.angle = angle
    msg.velocity = speed
    pub.publish(msg)
    msg = drive_param()
    angle = 0.
    speed = 0.
    msg.angle = angle
    msg.velocity = speed
    pub.publish(msg)
    msg = drive_param()
    angle = 0.
    speed = 0.
    msg.angle = angle
    msg.velocity = speed
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("pid_controller", anonymous=True)
    rospy.Subscriber(
        "path_planner",
        Float64MultiArray,
        desired_track)
    rospy.Subscriber("slam_out_pose", PoseStamped, control)
    """
    time.sleep(10)
    PROCNAME = 'master_test'

    for proc in psutil.process_iter():
        if proc.name == PROCNAME:
            proc_id = proc.pid
            break
    os.kill(proc_id, signal.SIGINT)
    """

    rospy.on_shutdown(myhook)
    rospy.spin()




