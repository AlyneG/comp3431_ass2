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
        self.pub = rospy.Publisher('stop',String,queue_size=1)
        self.stop = None
    def stop_detect(self, image):            
        img = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        cv2.imshow("s",img)
        mask_red = cv2.inRange(hsv, (175, 70, 50), (200, 255, 255))
        #mask_red = cv2.inRange(img,(40,30,120),(100,50,160))
        h,w,_ = img.shape

        imask_red = mask_red>0
        red = numpy.zeros_like(img, numpy.uint8)
        red[imask_red] = img[imask_red]
        r = red[:,:,2]
        ret, thresh = cv2.threshold(r, 0, 255, cv2.THRESH_BINARY)
        image,contours,_=cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print(len(contours))
        thresh = cv2.drawContours(thresh, contours, -1, (0,255,0), 1)
        max_area = 0
        for contour in contours:
            rect = cv2.boundingRect(contour)
            area = rect[2] * rect[3]
            max_area = max(area,max_area)
        proportion = max_area/(h*w*1.0)
    
        if(proportion >= 0.02):
            print("stop sign detect")
            self.pub.publish("yes")
        else:
            self.pub.publish("no")


        cv2.imshow("window", thresh)
        cv2.waitKey(3)
rospy.init_node('stop')
stop = Stop()
rospy.spin()
