#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge, numpy
from datetime import datetime, timedelta
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class Intersection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.intersection_detect)
        self.pub = rospy.Publisher('intersection',String,queue_size=1)
        self.stop = None
    def intersection_detect(self, image):
        if(self.stop != None and datetime.now() < self.stop-timedelta(seconds=10)):
            return
        elif(self.stop != None and datetime.now() >= self.stop-timedelta(seconds=10) and datetime.now()<self.stop):
            self.pub.publish("no")
            return
        else:
            self.stop = None
            self.pub.publish("no")


            
        image = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        #change perspective
        rows, cols = image.shape[:2]
        rows-=1
        cols-=1
        src_points = numpy.float32([[0,0],[cols, 0],[0,rows], [cols,rows]])
        dst_points = numpy.float32([[0,10],[cols,10], [int(cols*1/7),rows], [int((cols)*6/7),rows]])
        affine_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        image = cv2.warpPerspective(image, affine_matrix, (cols+1,rows+1))
        image[image==0] = 255
        intersection = False
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _,mask = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
        h, w, d = image.shape
        search_top = int(8.5*h/10)
        search_bot = int(h) 
        it = 10
        dilate = cv2.dilate(mask,None, iterations=it)
        mask = cv2.erode(dilate,None, iterations=it)
        mask = 255 - mask
        mask[search_bot:h, 0:w] = 0
        mask[0:search_top, 0:w] = 0
        number = numpy.count_nonzero(mask == 255)*1.0
        total = h*w*1.0
        prop = number/total
        #print(number,total,prop)
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (100,25)
        fontScale              = 0.5
        fontColor              = (255,255,255)
        lineType               = 2
        cv2.putText(mask,'{0:2f}'.format(prop), 
        bottomLeftCornerOfText, 
        font, 
        fontScale,
        fontColor,
        lineType)
        if(prop >= 0.04 and prop <= 0.05):
            print("intersection detect")
            self.stop = datetime.now()+timedelta(seconds=13)
            self.pub.publish("yes")
        else:
            self.pub.publish("no")

        
        #cv2.imshow("window", mask)
        cv2.waitKey(3)


rospy.init_node('intersection')
inters = Intersection()
rospy.spin()
