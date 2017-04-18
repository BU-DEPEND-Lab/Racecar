#!/usr/bin/env python

import rospy
from race.msg import odometry
from geometry_msgs.msg import PoseStamped
import sys
import serial

global last_pose

# callback function on occurance of drive parameters(angle & velocity)
def callback(data):
    global last_pose
    last_pose = data

if __name__ == '__main__':
    global last_pose
    rospy.init_node('fakeGPS', anonymous=True)
    rate = rospy.Rate(5)
    pub = rospy.Publisher('slam_out_pose2', PoseStamped, queue_size=1)
    rospy.Subscriber("slam_out_pose", PoseStamped, callback)
    last_pose = PoseStamped()
    while not rospy.is_shutdown():
        pub.publish(last_pose)
        rate.sleep()
    


