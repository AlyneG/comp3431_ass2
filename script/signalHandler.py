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
        self.turnEndTime = None

        self.max_pink = 0
        self.pink_x = None
        self.pink_y = None
        self.max_yellow = 0
        self.yellow_x = None

        self.yellow_y = None

        self.seen = None
    def signal(self,image):
        if(self.seen != None and datetime.now() < self.seen):
            return
        elif(self.seen != None and datetime.now() >= self.seen):
            self.pub.publish("no")
            return
        image = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

        # Set range for pink color and
        # define mask
        pink_lower = np.array([135, 70, 150], np.uint8)
        pink_upper = np.array([180, 255, 255], np.uint8)
        pink_mask = cv2.inRange(hsv, pink_lower, pink_upper)

        # Set range for yellow color and
        # define mask
        yellow_lower = np.array([20, 150, 100], np.uint8)
        yellow_upper = np.array([40, 255, 255], np.uint8)
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        # For pink color
        pink_mask = cv2.dilate(pink_mask, kernel)
        res_pink = cv2.bitwise_and(image, image, mask = pink_mask)


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

                if(area > self.max_yellow):
                    self.yellow_x = x
                    self.yellow_y = y
                    self.max_yellow = area
        left = None
        if(self.max_pink > 0 and self.max_yellow > 0):
            if(self.yellow_y > self.pink_y):
                #print("Turn left")
                self.pub.publish("left")
            else:
                #print("Turn right")
                self.pub.publish("right")
        else:
            self.pub.publish("Non")
        #cv2.imshow('color',image)
        cv2.waitKey(3)

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
