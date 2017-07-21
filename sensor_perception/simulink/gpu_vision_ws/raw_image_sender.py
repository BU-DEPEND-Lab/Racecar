#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cap = cv2.VideoCapture(0)



def sender():
    pub = rospy.Publisher('image_bridge', Image, queue_size=1)
    rospy.init_node('image_sender', anonymous=True)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        ret, img = cap.read()
        pub.publish(bridge.cv2_to_imgmsg(img, encoding="rgb8"))
        rate.sleep()
      
if __name__=='__main__':
    try:
        sender()
    except rospy.ROSInterruptException:
        pass
    cap.release()        
  
