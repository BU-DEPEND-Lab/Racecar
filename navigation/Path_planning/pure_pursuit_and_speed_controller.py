#!/usr/bin/env python

# PURE PURSUIT CONTROLLER for steering angle
# PD controller for speed control


# python includes
import numpy as np
import math
import time
import rospy
import tf

# ROS messages
from racecar_control.msg import drive_param
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray

# published topics

#em_pub = rospy.Publisher('eStop', Bool, queue_size=10)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

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
global look_ahead_dist

prev_steer = 0.
prev_speed = 0.

flag = 0


look_ahead_dist = 1.0  # 1m look ahead distance


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

    def accelerate(self, rate):
        self.speed += rate

    def deccelerate(self, rate):
        self.speed -= rate


"""

Read incoming data from path planner

	>>> data" is a 1D array of x position, y position, speed, x position, y position , speed ... and so on.

	>>> note that speed is a percentage of max speed that is required

"""


def desired_track(data):
    global flag
    global path_y
    global path_x
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


""" calculate desired steering angle """


def calculate_desired_steer(carrot_x, carrot_y, curr_x, curr_y, curr_yaw):

    delta_x = carrot_x - curr_x
    # l = look ahead distance from robot to carrot
    l = np.sqrt((carrot_x - curr_x)**2 + (carrot_y - curr_y)**2)
    print ("l = ", l)

    # subtract 1.56 because of rotated x, y axes....refer to report
    theta = (np.arcsin(delta_x / l)) - 1.56

    print ("theta = ", theta)
    print ("curr_yaw = ", curr_yaw)

    desired_steer = curr_yaw - theta
    return desired_steer


""" PD controller for steering """


def PD_controller(desired_steer, diff_error):
    kp = 1
    kd = 0.09

    steerOutput = - (kp * desired_steer) - (kd * diff_error)

    if steerOutput < -0.4:
        steerOutput = -0.4
    elif steerOutput > 0.4:
        steerOutput = 0.4

    return steerOutput


""" open loop controller for speed """


def speedControl(speed_percentage, prev_speed):
    max_speed = 40
    max_speed_reverse = -40
    min_speed = 0

    desired_speed = speed_percentage * max_speed

    speed = 0.9 * prev_speed + 0.1 * desired_speed

    if speed < -max_speed_reverse:
        speed = -max_speed_reverse
    elif speed > max_speed:
        speed = max_speed

    return speed


""" Stop the car if it is within an acceptable distance from the goal point """


def goalCheck(goal_x, goal_y, curr_x, curr_y):
    goalRadius = np.sqrt((goal_x - curr_x)**2 + (goal_y - curr_y)**2)
    if goalRadius < 0.5:
        return True
    else:
        return False


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
    global look_ahead_dist

    myrobot = robot()

    """ Flag is set when path arrives from planner node"""
    while flag == 1:
        # Update pose
        curr_x, curr_y, yaw = poseUpdate(data)
        myrobot.setPose(curr_x, curr_y, yaw)

        goal_x = path_x[planner_coord - 1]
        goal_y = path_y[planner_coord - 1]
        goal_speed = speed[planner_coord - 1]

        if goalCheck(goal_x, goal_y, myrobot.x, myrobot.y):
            flag = 0
            angle = 0.
            speed = 0.
            print ("GOAL REACHED")
            pub.publish(msg)
            break

        # Find carrot point on path
        possible_l = [0] * planner_coord
        for i in range(0, planner_coord):
            possible_l[i] = abs((path_x[i] - myrobot.x)**2 +
                                (path_y[i] - myrobot.y)**2 - look_ahead_dist)

        j = possible_l.index(min(possible_l))

        carrot_x = path_x[j]
        carrot_y = path_y[j]
        print ("carrot_x = ", carrot_x, "carrot_y = ", carrot_y)

        # Find desired steer value
        desired_steer = calculate_desired_steer(
            carrot_x,
            carrot_y,
            myrobot.x,
            myrobot.y,
            myrobot.yaw)
        print ("desired_steer = ", desired_steer)

        # Calculate steering output to publish using pd controller
        diff_error = desired_steer - prev_steer

        steerOutput = PD_controller(desired_steer, diff_error)
        prev_steer = steerOutput

        steerOutput = -desired_steer

        # Speed control
        speed_percentage = speed[j]  # carrot speed
        speed = speedControl(speed_percentage[j], prev_speed)
        prev_speed = speed

        # Publish message
        msg.angle = steerOutput
        msg.velocity = speed
        print("flag = 1, vel = ", msg.speed, ", angle = ", msg.angle)
        pub.publish(msg)
        flag = 0

    # keep speed and steering to 0 if no plan arrives and flag is not set
    curr_x, curr_y, yaw = poseUpdate(data)
    myrobot.setPose(curr_x, curr_y, yaw)

    angle = 0.
    speed = 0.
    msg.angle = angle
    msg.velocity = speed
    pub.publish(msg)
    # rospy.Rate(10)


if __name__ == "__main__":
    rospy.init_node("pid_controller", anonymous=True)
    rospy.Subscriber(
        "path_planner",
        Float64MultiArray,
        desired_track)
    rospy.Subscriber("slam_out_pose", PoseStamped, control)

    rospy.spin()


'''

def emergencyStop():
	t_end = time.time() + 5
	while time.time() < t_end:
		#do whatever you want to do
	em_pub.publish(True) # find a way to manually chage to false
'''
