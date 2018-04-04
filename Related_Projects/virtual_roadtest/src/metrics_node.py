#!/usr/bin/env python


""" 
Author: Muhammad Zuhayr Raghib  12/09/2017

ROS node to accumulate data continuously at runtime

outputs : prints calculated metrcis to stdout which is read by parent process 
          for calculating the fitness value considered in the genetic algorithm

"""


# python includes
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan

# ROS messages
from geometry_msgs.msg import Point



class Metrics:
    def __init__(self):
        # steer control variables:
        self.minDist = 0.
        self.maxSpeed = 0.
        self.AvgSpeed = 0.
        self.AvgSteering = 0.

#global variables
global myMetrics
global angles
global speeds


angles = []
speeds = []
myMetrics = Metrics()

# Get min distance from the right/ left side of the car form an obstacle
def getRange(data):    
    global myMetrics
    
    mydata = data
    

    left = mydata.ranges[180]  #1 scan has 1080 indeces, each spaced by 0.25 degrees
    right = mydata.ranges[900]
    myMetrics.minDist = min(left, right)


def drive_data(data):
    global angles
    global speeds
    global myMetrics
    
    mydata = data
    
    if(mydata.x == 0):
        return 0

    max_speed = myMetrics.maxSpeed
    if(mydata.x > max_speed):
        myMetrics.maxSpeed = mydata.x

    angles.append(abs(mydata.y))
    speeds.append(mydata.x)


    
def timeCompletion(data):
    pass



def myhook(): # print latest values of all metrics to be read by GA algorithm
    global myMetrics
    global angles
    # average angle
    if(len(angles)>=1):
        myMetrics.AvgSteering = np.mean(angles)
        
    # average speed
    if(len(speeds)>=1):
        myMetrics.AvgSpeed = np.mean(speeds)        
    
    print (myMetrics.minDist)
    print (myMetrics.maxSpeed)
    print (myMetrics.AvgSpeed)
    print (myMetrics.AvgSteering)

if __name__ == "__main__":
    rospy.init_node('metrics_node',anonymous = True)
    
    #Lidar data
    rospy.Subscriber("scan",LaserScan,getRange)
    
    # Drive parameters
    rospy.Subscriber("drive_parameters", Point, drive_data)

    rospy.on_shutdown(myhook)
    rospy.spin()



""" 
#May be useful
import roslaunch
import time
import rospy


rospy.init_node('en_Mapping', anonymous=True)
#rospy.on_shutdown(self.shutdown)

uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/f1/catkin_ws/src/racecar_simulator/racecar_gazebo/launch/racecar_tunnel.launch"])

launch.start()
time.sleep(15)

launch.shutdown()
"""