#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
pub = rospy.Publisher('image_bridge', Image, queue_size=1)


def callback(img):
    resized = cv2.resize(bridge.imgmsg_to_cv2(img, desired_encoding="passthrough"), (960, 540))
    pub.publish(bridge.cv2_to_imgmsg(resized, encoding="rgb8"))
    

def main():
    rospy.init_node('image_converter', anonymous=True)
    sub = rospy.Subscriber('/zed/left/image_raw_color', Image, callback)
    rospy.spin()
      
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass     
  
