#!/usr/bin/env python

# ignore z axis

# x is y and y is x!!
import rospy
from math import *
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
from sympy import Point, Segment
#from tf.transformations import euler_from_quaternion  #use when considering orientations to convert from quaternions

path = []

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

global prev_error
global velocity
global kp 
global kd
global flag
global length



global path_y
global path_x


velocity = 0.0  #constant
kp = 14.0
kd = 0.09
#ki = 0.0 
prev_error = 0
flag = 0
length = 0  #size of array of coordinates

'''create robot and initialize location/orientation to 0, 0, 0'''
class robot:  
    def __init__(self): 
        self.x = 0.0
        self.y = 0.0    
        self.curr_pose = Point(0.0, 0.0)
    '''sets a robot pose (NO ORIENTATION IS CONSIDERED AS OF NOW)'''
    def set(self, new_x, new_y):  
        self.x = new_x 
        self.y = new_y 
        self.curr_pose = Point(new_x , new_y)
          
def desired_track(data):
    '''put incoming data into a suitable datastructure'''
    global length
    global flag
    global path_y
    global path_x
        
        
    length = np.size(data.data)
    p = np.reshape(data.data,((length/2),2))

    path_y = [0.0 for ii in range(length/2)]
    path_x = [0.0 for ii in range(length/2)]
    
    for i in range(0,length/2):
        path_y[i] = p[i][1]
        path_x[i] = p[i][0]

    flag = 1

def control(data):    
    msg = drive_param(); 

    global prev_error
    global velocity
    global kp 
    global kd
    global flag
    global length
    global path_y
    global path_x
        
    myrobot = robot() 
    myrobot.set(data.pose.position.x , data.pose.position.y)     #update current pose
    
    ''' flag is set when path arrives from planner node'''    
    if flag == 1:        
        '''find distance from robot and the desired track (cross track error)'''
        for index in range(0, ((length/2)-1)):
            myrobot.set(data.pose.position.x , data.pose.position.y)   #update current pose 
            print ( "curr_y = " , myrobot.y, "curr_x = " , myrobot.x)
            print ( "path_y = " , path_y[index], "path_x = " , path_x[index])
            p1 = Point(path_y[index], path_x[index])   # points from desired path
            p2 = Point(path_y[index+1], path_x[index+1])
            s = Segment(p1, p2)   
            error = -float(s.distance(myrobot.curr_pose))  # x in the equation  -->  -x-Lsin(theta)
            if (myrobot.y < path_y[index]):
                error = -error      #-error = left turn, +ve error = right turn
            print ("error = " , error)

            '''calculate steering angle using pid controller'''
            #int_error += error     # integral not used
            diff_error = error - prev_error    # differential
            
            angle = - (kp * error) - (kd * diff_error) #   - (ki * int_CTE)        
        
            if angle < -100 :
                angle = -100 
            elif angle > 100 :
                angle = 100      
            prev_error = error 	
        	
            msg.angle = angle
            velocity = 10.0
            msg.velocity = velocity
            #print("flag = 1, vel = " , msg.velocity, ", angle = " , msg.angle)
            pub.publish(msg) 
        flag = 0
    
    ''' keep vel and steering to 0 if no plan arrives and flag is not set'''
    angle = 0
    velocity = 0.0
    msg.angle = angle
    msg.velocity = velocity  # constant velocity
    print("flag = 0, vel = ", msg.velocity, ", angle = " , msg.angle)
    pub.publish(msg) 


if __name__ == "__main__":
    rospy.init_node("pid_controller", anonymous=True)
    rospy.Subscriber("path_planner", Float64MultiArray, desired_track) # from path planner
    rospy.Subscriber("slam_out_pose", PoseStamped, control) # from slam
    rospy.spin()  
        