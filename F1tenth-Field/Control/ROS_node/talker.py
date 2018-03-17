#!/usr/bin/env python

import rospy
from race.msg import drive_values
#from race.msg import drive_param
from geometry_msgs.msg import Point
from std_msgs.msg import Bool  
import numpy as np

pub = rospy.Publisher('drive_pwm', drive_values, queue_size=10)
em_pub = rospy.Publisher('eStop', Bool, queue_size=10)

# function to map from one range to another, similar to arduino
def arduino_map(x, in_min, in_max, out_min, out_max):
	ret =  np.int16((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
	return ret

# callback function on occurance of drive parameters(angle & velocity)
def callback(data):
    msg = drive_values()
    velocity = data.x
    angle = data.y
    # forward and backward offset
    if (velocity < 0.0):
        velocity -= 10.1
    elif (velocity > 0.0):  
        velocity += 6.1
	# Do the computation
    pwm1 = arduino_map(velocity,-100,100,6554,13108);
    pwm2 = arduino_map(angle,-0.78,0.78,7619,12836);
    msg.pwm_drive = pwm1
    msg.pwm_angle = pwm2
    print("pwm drive:",pwm1,"pwm angle:",pwm2)
    pub.publish(msg)


def talker():
	rospy.init_node('serial_talker', anonymous=True)
	em_pub.publish(False)
	rospy.Subscriber("drive_parameters", Point, callback)
	rate = rospy.Rate(30)
	while(not rospy.is_shutdown()):
		rospy.spin()

if __name__ == '__main__':
	print("Serial talker initialized")
	talker()
