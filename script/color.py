#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

class colorDetection:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image',Image, self.color_callback)

        self.detect_color_pub = rospy.Publisher('control/color', Float64, queue_size=1)

        self.color_code = Float64()

        self.pink_detected = False
        self.green_detected = False
        self.blue_detected = False
        self.yellow_detected = False

    
    def color_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        # Set range for pink color and
        # define mask 
        pink_lower = np.array([150, 120, 111], np.uint8) 
        pink_upper = np.array([160, 255, 255], np.uint8) 
        pink_mask = cv2.inRange(hsv, pink_lower, pink_upper) 
  
        # Set range for green color and  
        # define mask 
        green_lower = np.array([40, 150, 150], np.uint8) 
        green_upper = np.array([90, 255, 255], np.uint8) 
        green_mask = cv2.inRange(hsv, green_lower, green_upper) 
  
        # Set range for blue color and 
        # define mask 
        blue_lower = np.array([90, 100, 100], np.uint8) 
        blue_upper = np.array([110, 255, 255], np.uint8) 
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
    
        # Set range for yellow color and
        # define mask
        yellow_lower = np.array([20, 150, 150], np.uint8)
        yellow_upper = np.array([30, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # For pink color 
        pink_mask = cv2.dilate(pink_mask, kernel) 
        res_pink = cv2.bitwise_and(image, image, mask = pink_mask) 

        #plt.imsave('red.jpg', res_pink)
    
        # For green color 
        green_mask = cv2.dilate(green_mask, kernel) 
        res_green = cv2.bitwise_and(image, image, 
                                    mask = green_mask) 
        #plt.imsave('green.jpg', res_green)
    
        # For blue color 
        blue_mask = cv2.dilate(blue_mask, kernel) 
        res_blue = cv2.bitwise_and(image, image, 
                                   mask = blue_mask) 
   
        #plt.imsave('blue.jpg', res_blue)
    
        # For yellow color
        yellow_mask = cv2.dilate(yellow_mask, kernel)
        res_yellow = cv2.bitwise_and(image, image,
                                     mask = yellow_mask)
    
    
        # Creating contour to track pink color 
        img, contours_pink, hierarchy = cv2.findContours(pink_mask, 
                                               cv2.RETR_TREE, 
                                               cv2.CHAIN_APPROX_SIMPLE) 

        
        for pic, contour in enumerate(contours_pink): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y),  
                                         (x + w, y + h),  
                                         (229, 92, 190), 2) 
              
                cv2.putText(image, "Pink Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (229, 92, 190))

                print('Pick Color Detected')
                self.pink_detected = True
  

        # Creating contour to track green color 
        img, contours_green, hierarchy = cv2.findContours(green_mask, 
                                               cv2.RETR_TREE, 
                                               cv2.CHAIN_APPROX_SIMPLE) 
     
        for pic, contour in enumerate(contours_green): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y),  
                                           (x + w, y + h), 
                                           (0, 255, 0), 2) 
              
                cv2.putText(image, "Green Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX,  
                            1.0, (0, 255, 0)) 
            
                print('Green Color Detected')
                self.green_detected = True
            
  
        # Creating contour to track blue color 
        img, contours_blue, hierarchy = cv2.findContours(blue_mask, 
                                               cv2.RETR_TREE, 
                                               cv2.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours_blue): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y), 
                                           (x + w, y + h), 
                                           (0, 0, 255), 2) 
              
                cv2.putText(image, "Blue Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            1.0, (0, 0, 255)) 
            
                print('Blue Color Detected')
                self.blue_detected = True
    
        # Creating contour to track yellow color
        img, contours_yellow, hierarchy = cv2.findContours(yellow_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
    
        for pic, contour in enumerate(contours_yellow): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y), 
                                           (x + w, y + h), 
                                           (255, 254, 50), 2) 
              
                cv2.putText(image, "Yellow Colour", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 
                            1.0, (255, 254, 50))
            
                print('Yellow Color Detected')
                self.yellow_detected = True
            
        if(self.pink_detected and self.yellow_detected):
            self.color_code.data = 0
            self.detect_color_pub.publish(self.color_code)
            pass

        
        if(self.pink_detected and self.green_detected):
            self.color_code.data = 1
            self.detect_color_pub.publish(self.color_code)
            pass
        
        if(self.pink_detected and self.blue_detected):
            self.color_code.data = 2
            self.detect_color_pub.publish(self.color_code)
            pass

        if(not self.pink_detected and not self.green_detected and not self.blue_detected and not self.yellow_detected):
            self.color_code.data = -1
            self.detect_color_pub.publish(self.color_code)

        cv2.imshow('color',image)
        cv2.waitKey(3)



rospy.init_node('colorDetection')
colorDetection = colorDetection()
rospy.spin()
