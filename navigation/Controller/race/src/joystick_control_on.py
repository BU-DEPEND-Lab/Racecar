#!/usr/bin/env python
import rospy
from race.msg import drive_param
from sensor_msgs.msg import Joy
import subprocess
import datetime
import os
import signal
global forward
global left
global a
forward = 0
left = 0
rospy.init_node('keyboard_talker', anonymous=True)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)

def terminate_process_and_children(p):
  import psutil
  process = psutil.Process(p.pid)
  for sub_process in process.get_children(recursive=True):
      sub_process.send_signal(signal.SIGINT)
  p.wait()  # we wait for children to terminate
  p.terminate()

def control(data):
	global forward
	global left
        global a
	button = data.buttons
	axes = data.axes
        ''' if button[1] == 1:
                forward = forward + 1;
        elif button[2] == 1:
                forward = forward - 1;
        if button[4] == 1:
                left = left - 5;
        elif button[5] == 1:
                left = left + 5;
        if button[3] == 1:
                left = 0
                forward = 0
	if button[7] == 1:
                cmd = "rosbag record -o /media/ubuntu/bc269b87-e580-431d-8779-a423f17ff2cf2/ -a"
		d = ['rosbag', 'record','-o', '/media/ubuntu/bc269b87-e580-431d-8779-a423f17ff2cf2/', '-a' ]
		rospy.loginfo("recording_bags")
        	a = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell =True) 
		rospy.loginfo(datetime.datetime.now())
	if button[0] == 1:
		rospy.loginfo('stopped recording')
		os.kill(a.pid,signal.SIGTERM)'''
	
	left = axes[0] * (-100)
	accelerate = abs(axes[4] - 1) * 5
	decelerate = abs(axes[5] - 1) * 5
	forward = accelerate - decelerate
        if button[7] == 1:
                cmd = "rosbag record -O /media/ubuntu/bc269b87-e580-431d-8779-a423f17ff2cf2/joy_adaptor_on ekf_pub slam_out_pose"
                rospy.loginfo("recording_bags")
                a = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid) 
                rospy.loginfo(datetime.datetime.now())
	if button[3] == 1:
		left = 0
		forward = 0 
        if button[0] == 1:
		print('absds')
                rospy.loginfo('stopped recording')
		#os.killpg(os.getpgid(a.pid), signal.SIGINT)
		terminate_process_and_children(a)
	msg = drive_param()
        msg.velocity = forward 
        msg.angle = left
        pub.publish(msg)

if __name__ == '__main__':
	rospy.init_node('keyboard_talker', anonymous=True)
        rospy.Subscriber("joy", Joy, control)
        rospy.spin()
