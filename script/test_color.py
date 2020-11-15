#!/usr/bin/env python

import cv2, cv_bridge, numpy as np

class colorDetection:

    def __init__(self):

        self.pink_detected = False
        self.green_detected = False
        self.blue_detected = False
        self.yellow_detected = False

        self.max_pink = 0
        self.pink_x = None
        self.pink_y = None
        self.max_green = 0
        self.green_x = None
        self.green_y = None
        self.max_blue = 0
        self.blue_x = None
        self.blue_y = None
        self.max_yellow = 0
        self.yellow_x = None
        self.yellow_y = None

    def color_callback(self):
        print("I am here")

        #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        image = cv2.imread('MicrosoftTeams-image.png')

        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        # Set range for pink color and
        # define mask
        pink_lower = np.array([140, 100, 150], np.uint8)
        pink_upper = np.array([170, 255, 255], np.uint8)
        pink_mask = cv2.inRange(hsv, pink_lower, pink_upper)

        # Set range for green color and
        # define mask
        green_lower = np.array([35, 100, 50], np.uint8)
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
        img, contours, hierarchy = cv2.findContours(pink_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        self.max_pink = 0
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                image = cv2.rectangle(image, (x, y),
                                         (x + w, y + h),
                                         (229, 92, 190), 2)

                cv2.putText(image, "Pink Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                            (229, 92, 190))

                print('Pink Color Detected')
                self.pink_detected = True
                #finds the largest area of colour
                #theoretically, this should be the closest to the robot
                if(area > self.max_pink):
                    self.pink_x = x
                    self.pink_y = y
                    self.max_pink = area
        cv2.putText(image, "MAX PINK", (self.pink_x - 5, self.pink_y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (229, 92, 190))

        # Creating contour to track green color
        img, contours, hierarchy = cv2.findContours(green_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        self.max_green = 0
        for pic, contour in enumerate(contours):
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
                if(area > self.max_green):
                    self.green_x = x
                    self.green_y = y
                    self.max_green = area


        # Creating contour to track blue color
        img, contours, hierarchy = cv2.findContours(blue_mask,
                                               cv2.RETR_TREE,
                                               cv2.CHAIN_APPROX_SIMPLE)
        self.max_blue = 0
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                image = cv2.rectangle(image, (x, y),
                                           (x + w, y + h),
                                           (255, 0, 0), 2)

                cv2.putText(image, "Blue Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 0, 0))

                print('Blue Color Detected')
                self.blue_detected = True
                if(area > self.max_blue):
                    self.blue_x = x
                    self.blue_y = y
                    self.max_blue = area

        # Creating contour to track yellow color
        img, contours, hierarchy = cv2.findContours(yellow_mask,
                                                cv2.RETR_TREE,
                                                cv2.CHAIN_APPROX_SIMPLE)
        self.max_yellow = 0
        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area > 300):
                x, y, w, h = cv2.boundingRect(contour)
                image = cv2.rectangle(image, (x, y),
                                           (x + w, y + h),
                                           (0, 255, 255), 2)

                cv2.putText(image, "Yellow Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 255, 255))

                print('Yellow Color Detected')
                self.yellow_detected = True
                if(area > self.max_yellow):
                    self.yellow_x = x
                    self.yellow_y = y
                    self.max_yellow = area

        # if(self.pink_detected and self.yellow_detected):
        #
        #     #msg_stop = Bool()
        #     #msg_stop.data = True
        #     #self.stop_pub.publish(msg_stop)
        #
        #     print("Pink and Yellow")
        #     print("Pink max: ")
        #     print(self.max_pink)
        #     print("Yellow max: ")
        #     print(self.max_yellow)
        #
        #
        # if(self.pink_detected and self.green_detected):
        #     print("Pink and Green")
        #     print("Pink max: ")
        #     print(self.max_pink)
        #     print("Green max: ")
        #     print(self.max_green)
        #
        # if(self.pink_detected and self.blue_detected):
        #     print("Pink and Blue")
        #     print("Pink max: ")
        #     print(self.max_pink)
        #     print("Blue max: ")
        #     print(self.max_blue)

        if(self.max_pink > 0):
            if(self.max_green > self.max_blue):
                if(self.max_green > self.max_yellow):
                    #green has the largest area
                    print("green has the largest area")
                    print(self.green_x)
                    print(self.green_y)
                    print(self.pink_x)
                    print(self.pink_y)
                    #if(self.pink_x > )
                elif(self.max_yellow >= self.max_green):
                    #yellow has the largest area
                    print("yellow has the largest area")
                    print(self.yellow_x)
                    print(self.yellow_y)
                    print(self.pink_x)
                    print(self.pink_y)
            else:
                if(self.max_blue > self.max_yellow):
                    #blue has the largest area
                    print("blue has the largest area")
                    print(self.blue_x)
                    print(self.blue_y)
                    print(self.pink_x)
                    print(self.pink_y)
                elif(self.max_yellow >= self.max_green):
                    #yellow has the largest area
                    print("yellow has the largest area")
                    print(self.yellow_x)
                    print(self.yellow_y)
                    print(self.pink_x)
                    print(self.pink_y)
        # Program Termination
        #plt.imsave('color.jpg', img)
        imgS = cv2.resize(image, (540, 960))
        cv2.imshow('color',imgS)
        hsvS = cv2.resize(hsv, (540, 960))
        cv2.imshow('hsv', hsvS)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

colour = colorDetection()
colour.color_callback()
