#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool


class detectStopSign:
    
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image',Image, self.stop_sign_callback)
        
        self.detect_stopSign_pub = rospy.Publisher('detect/stopSign', Bool, queue_size=1)
        
        self.bool = Bool()

    def stop_sign_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)


        red_lower = numpy.array([0, 120, 111], numpy.uint8) 
        red_upper = numpy.array([10, 255, 255], numpy.uint8) 
        red_mask = cv2.inRange(hsv, red_lower, red_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        red_mask = cv2.dilate(red_mask, kernel) 
        
        h, w, d = image.shape
        left = int(0.3*w)
        right = int(0.7*w)
        red_mask[:,0:left] = 0
        red_mask[:,right:w] = 0

        red_pink = cv2.bitwise_and(image, image, mask = red_mask)

        img, contours_red, hierarchy = cv2.findContours(red_mask, 
                                               cv2.RETR_TREE, 
                                               cv2.CHAIN_APPROX_SIMPLE) 

        if(len(contours_red) == 0):
            self.bool.data = False
            self.detect_stopSign_pub.publish(self.bool)

        for pic, contour in enumerate(contours_red): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y),  
                                         (x + w, y + h),  
                                         (229, 92, 190), 2) 
              
                cv2.putText(image, "Red Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (229, 92, 190))

                print('Red Color Detected')
                self.bool.data = True
                self.detect_stopSign_pub.publish(self.bool)
                #self.pink_detected = True
        '''
        #get red part
        blue_lower=numpy.array([0, 123, 100])
        blue_upper=numpy.array([5, 255, 255])
        mask=cv2.inRange(hsv,blue_lower,blue_upper)
   
        #blur
        blurred=cv2.blur(mask,(9,9))

        ret,binary=cv2.threshold(blurred,127,255,cv2.THRESH_BINARY)

        #eliminate gaps
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        #erode and dliate

        erode=cv2.erode(closed,None,iterations=4)
        dilate=cv2.dilate(erode,None,iterations=4)

        # find contours
        img,contours, hierarchy=cv2.findContours(dilate.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        res=img.copy()

        if(len(contours) == 0):
            print('No stop sign')
            self.bool.data = False
            self.detect_stopSign_pub.publish(self.bool)
        else:
            for con in contours:
                #make a rectangle contour
                rect=cv2.minAreaRect(con)
                #trans the rectangle to box
                box=numpy.int0(cv2.boxPoints(rect))
                #draw the contour on the original pic 
                cv2.drawContours(res,[box],-1,(0,0,255),2)
    
                #calculate
                h1=max([box][0][0][1],[box][0][1][1],[box][0][2][1],[box][0][3][1])
                h2=min([box][0][0][1],[box][0][1][1],[box][0][2][1],[box][0][3][1])
                l1=max([box][0][0][0],[box][0][1][0],[box][0][2][0],[box][0][3][0])
                l2=min([box][0][0][0],[box][0][1][0],[box][0][2][0],[box][0][3][0])

                #double check
                if h1-h2>0 and l1-l2>0:
            
                    temp=img[h2:h1,l2:l1]
                    print('STOP SIGN DETECTED')
                
                    self.bool.data = True
                    self.detect_stopSign_pub.publish(self.bool)
                
        '''
        cv2.imshow('res',red_mask)
        cv2.waitKey(3)

    
    
rospy.init_node('stopSignDetection')
stopSignDetection = detectStopSign()
rospy.spin()
