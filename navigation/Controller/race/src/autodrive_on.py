#!/usr/bin/env python
import rospy
from race.msg import drive_param
from sensor_msgs.msg import Joy
import subprocess
import datetime
import time
import os
import signal

def terminate_process_and_children(p):
  import psutil
  process = psutil.Process(p.pid)
  for sub_process in process.get_children(recursive=True):
      sub_process.send_signal(signal.SIGINT)
  p.wait()  # we wait for children to terminate
  p.terminate()

def terminate_process(p):
  p.send_signal(signal.SIGINT)

global record_proc

if __name__ == '__main__':
	rospy.init_node('keyboard_talker', anonymous=True)
        rate = rospy.Rate(30)
        pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
        init_time = time.time()                
        cmd = "rosbag record -O /media/ubuntu/bc269b87-e580-431d-8779-a423f17ff2cf2/autodrive_adaptor_on ekf_pub slam_out_pose" # ZED: zed/odom
        recording = False
        stop_recording = True
	while not rospy.is_shutdown():
          msg = drive_param()
          msg.angle = 0
          duration = time.time() - init_time
          if duration > 15 and duration < 18:
            msg.velocity = 10
            if not recording:
              record_proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)         
              recording = True
          elif duration >= 18:
            msg.velocity = 0
            if stop_recording and record_proc:
              try:
                terminate_process_and_children(record_proc)
              except OSError:
                print record_proc
                print record_proc.pid
              stop_recording = False
          else:
            msg.velocity = 0
          pub.publish(msg)
          rate.sleep()
          
