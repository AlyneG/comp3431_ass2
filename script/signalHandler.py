#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge
import numpy as np
from datetime import datetime, timedelta
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import tf
import time

class Signal:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.signal)
        self.pub = rospy.Publisher('turn',String,queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.stop = None


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
    def signal(self,image):
        image = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        #image = cv2.imread('MicrosoftTeams-image.png',desired_encoding='bgr8')
        #h, w, d = image.shape
        #search_top = int(h/10)

        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        # Set range for pink color and
        # define mask
        pink_lower = np.array([135, 70, 150], np.uint8)
        pink_upper = np.array([180, 255, 255], np.uint8)
        pink_mask = cv2.inRange(hsv, pink_lower, pink_upper)

        # Set range for green color and
        # define mask
        green_lower = np.array([40, 70, 50], np.uint8)
        green_upper = np.array([80, 255, 255], np.uint8)
        green_mask = cv2.inRange(hsv, green_lower, green_upper)

        # Set range for blue color and
        # define mask
        blue_lower = np.array([90, 100, 50], np.uint8)
        blue_upper = np.array([120, 255, 255], np.uint8)
        blue_mask = cv2.inRange(hsv, blue_lower, blue_upper)
        #blue_mask[0:search_top, 0:w] = 0 #stops it from detecting the blue from the window

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

                if(area > self.max_pink):
                    self.pink_x = x
                    self.pink_y = y
                    self.max_pink = area

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

                # print('Green Color Detected')
                # self.green_detected = True
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
                                           (0, 0, 255), 2)

                cv2.putText(image, "Blue Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255))
                #
                # print('Blue Color Detected')
                # self.blue_detected = True
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
                                           (255, 254, 50), 2)

                cv2.putText(image, "Yellow Colour", (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (255, 254, 50))

                # print('Yellow Color Detected')
                # self.yellow_detected = True
                if(area > self.max_yellow):
                    self.yellow_x = x
                    self.yellow_y = y
                    self.max_yellow = area

        if(self.max_pink > 0):
            if(self.max_green > self.max_blue):
                if(self.max_green > self.max_yellow):
                    #green has the largest area
                    #look at position of green relative to pink
                    print("green has the largest area")
                    print(self.green_x)
                    print(self.green_y)
                    print(self.pink_x)
                    print(self.pink_y)
                    if(self.green_y > self.pink_y):
                        print("Pink is on top")
                    else:
                        print("Green is on top")
                    #if(self.pink_x > )
                elif(self.max_yellow >= self.max_green):
                    #yellow has the largest area
                    #look at position of yellow relative to pink
                    print("yellow has the largest area")
                    print(self.yellow_x)
                    print(self.yellow_y)
                    print(self.pink_x)
                    print(self.pink_y)
                    if(self.yellow_y > self.pink_y):
                        print("Pink is on top")
                    else:
                        print("Yellow is on top")
            else:
                if(self.max_blue > self.max_yellow):
                    #blue has the largest area
                    #look at position of blue relative to pink
                    print("blue has the largest area")
                    print(self.blue_x)
                    print(self.blue_y)
                    print(self.pink_x)
                    print(self.pink_y)
                    if(self.blue_y > self.pink_y):
                        print("Pink is on top")
                    else:
                        print("Blue is on top")
                elif(self.max_yellow >= self.max_green):
                    #yellow has the largest area
                    #look at position of yellow relative to pink
                    print("yellow has the largest area")
                    print(self.yellow_x)
                    print(self.yellow_y)
                    print(self.pink_x)
                    print(self.pink_y)
                    if(self.yellow_y > self.pink_y):
                        print("Pink is on top")
                    else:
                        print("Yellow is on top")
        # Program Termination
        #plt.imsave('color.jpg', image)
        cv2.imshow('color',image)
        cv2.waitKey(3)

    def start_turn(self):
        now = datetime.now()
        stop = now + timedelta(seconds=1.5)
        while datetime.now()<stop:
            signal.turn(False)
            self.pub.publish("yes")
        signal.stop_turn()
        self.pub.publish("no")
        

    def turn(self,left):
        twist = Twist()
        twist.linear.y = 0
        twist.linear.x = 0.1
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        if(left):
            twist.angular.z = 0.5
        else:
            twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)
    def stop_turn(self):
        twist = Twist()
        twist.linear.y = 0
        twist.linear.x = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

if __name__=='__main__':
    rospy.init_node('turn', anonymous=True)
    signal = Signal()
    rospy.spin()
    
    

'''
listener = tf.TransformListener()
listener.waitForTransform("/odom", "/map", rospy.Time(0), rospy.Duration(3))
position, orientation = listener.lookupTransform('/odom', '/map', rospy.Time(0))
client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
client.wait_for_server()

goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "map"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = position[-1]
goal.target_pose.pose.position.y = position[-1] + 0.1
goal.target_pose.pose.position.z = position[-1] + 0.5

goal.target_pose.pose.orientation.w = 1.0
client.send_goal(goal)
client.wait_for_result()
rospy.init_node('turn')
inters = Signal()
rospy.spin()'''

