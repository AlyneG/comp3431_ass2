#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge, numpy
from datetime import datetime, timedelta
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class Stop:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.stop_detect)
        self.pub = rospy.Publisher('stop',String,queue_size=10)
        self.stop = None
    def stop_detect(self, image):            
        img = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask_red = cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
    


        imask_red = mask_red>0
        
        red = numpy.zeros_like(img, numpy.uint8)
        red[imask_red] = img[imask_red]
        r = red[:,:,2]
        ret, thresh = cv2.threshold(r, 0, 255, cv2.THRESH_BINARY)
        _,contours,_=cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        print(len(contours))

        cv2.imshow("red",r)


        #cv2.imshow("window", mask)
        cv2.waitKey(3)
rospy.init_node('stop')
stop = Stop()
rospy.spin()
