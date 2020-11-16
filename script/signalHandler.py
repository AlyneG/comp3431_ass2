#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy, cv2, cv_bridge, numpy
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
    def signal(self,image):
        image = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        self.pub.publish("yes")        
        self.pub.publish("no")

rospy.init_node('s', anonymous=True)
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
'''
rospy.init_node('turn')
inters = Signal()
rospy.spin()'''

