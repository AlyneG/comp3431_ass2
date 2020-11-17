#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_msgs.msg import Float64
from std_msgs.msg import Bool

class colorDetection:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image',Image, self.color_callback)
        self.move_sub = rospy.Subscriber('control/move', Bool, self.move_callback)
        self.max_vel_pub = rospy.Publisher('control/max_vel', Float64, queue_size=1)
        self.stop_pub = rospy.Publisher('control/stop', Bool, queue_size=1)

        # self.pink_detected = False
        # self.green_detected = False
        # self.blue_detected = False
        # self.yellow_detected = False

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

    def color_callback(self, msg):

        



    def move_callback(self, msg):

        move = msg.data

        if(move):

            msg_max_vel = Float()
            msg_max_vel.data = 0.1
            self.max_vel_pub.publish(msg_max_vel)



rospy.init_node('colorDetection')
colorDetection = colorDetection()
rospy.spin()
