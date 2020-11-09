#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Bool

#Lode original pic
class detectStopSign:
    
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image, self.stop_sign_callback)
        self.intersection_sub = rospy.Subscriber('detect/intersection', Bool, self.intersection_callback)
        #self.move_sub = rospy.Subscriber('control/move', Bool, self.move_callback)

        self.max_vel_pub = rospy.Publisher('control/max_vel', Float64, queue_size=1)
        #self.stop_pub = rospy.Publisher('control/stop', Bool, queue_size=1)
        self.stop_sign_detected = False

    def stop_sign_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #print('img:',type(img),img.shape,img.dtype)
        #cv2.imshow('img',img)

        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        #cv2.imshow('hsv',hsv)

        #get red part
        blue_lower=numpy.array([0, 123, 100])
        blue_upper=numpy.array([5, 255, 255])
        mask=cv2.inRange(hsv,blue_lower,blue_upper)
        #print('mask',type(mask),mask.shape)
        #cv2.imshow('mask',mask)

        #blur
        blurred=cv2.blur(mask,(9,9))
        #cv2.imshow('blurred',blurred)
        #trans to binary
        ret,binary=cv2.threshold(blurred,127,255,cv2.THRESH_BINARY)
        #cv2.imshow('blurred binary',binary)
        #eliminate gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        #cv2.imshow('closed',closed)

        #erode and dliate

        erode=cv2.erode(closed,None,iterations=4)
        #cv2.imshow('erode',erode)
        dilate=cv2.dilate(erode,None,iterations=4)
        #cv2.imshow('dilate',dilate)

        # find contours
        img,contours, hierarchy=cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        #print('num of contours：',len(contours))
        i=0
        res=img.copy()
        if(len(contours) == 0):
            print('NO STOP SIGN')
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.1
            self.max_vel_pub.publish(msg_pub_max_vel)

        for con in contours:
            #make a rectangle contour
            rect=cv2.minAreaRect(con)
            #trans the rectangle to box
            box=numpy.int0(cv2.boxPoints(rect))
            #draw the contour on the original pic 
            cv2.drawContours(res,[box],-1,(0,0,255),2)
            #print([box])
            #calculate
            h1=max([box][0][0][1],[box][0][1][1],[box][0][2][1],[box][0][3][1])
            h2=min([box][0][0][1],[box][0][1][1],[box][0][2][1],[box][0][3][1])
            l1=max([box][0][0][0],[box][0][1][0],[box][0][2][0],[box][0][3][0])
            l2=min([box][0][0][0],[box][0][1][0],[box][0][2][0],[box][0][3][0])
            #print('h1',h1)
            #print('h2',h2)
            #print('l1',l1)
            #print('l2',l2)
            print('HAHAHA')
            #double check
            if h1-h2>0 and l1-l2>0:
            
                temp=img[h2:h1,l2:l1]
                i=i+1
                #show the sign part
                print('STOP SIGN DETECTED')
                self.stop_sign_detected = True
                #cv2.imshow('sign'+str(i),temp)

        #the result     
    #print(1)
        cv2.imshow('res',res)

        cv2.waitKey(3)
        #cv2.destroyAllWindows()
    
    def intersection_callback(self, msg):

        intersection_detected = msg.data

        if(intersection_detected and self.stop_sign_detected):
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0
            self.max_vel_pub.publish(msg_pub_max_vel)
            '''
            msg_pub_stop = Bool()
            msg_pub_stop.data = True
            self.stop_pub.publish(msg_pub_stop)
            #rospy.sleep(5)
            #msg_pub_max_vel = Float64()
            #msg_pub_max_vel.data = 0.3
            #self.max_vel_pub.publish(msg_pub_max_vel) 
            '''  
        '''
        if(not intersection_detected):
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.3
            self.max_vel_pub.publish(msg_pub_max_vel)  
    
        '''
    '''
    def move_callback(self, msg):

        move = msg.data

        if(move):
            msg_pub_max_vel = Float64()
            msg_pub_max_vel.data = 0.3
            self.max_vel_pub.publish(msg_pub_max_vel)
    '''         

rospy.init_node('stepSignDetection')
stopSignDetection = detectStopSign()
rospy.spin()
