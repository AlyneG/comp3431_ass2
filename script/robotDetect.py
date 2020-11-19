#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Fliter:
    def __init__(self):
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback)
        self.pub = rospy.Publisher('obstruct', String, queue_size=1)
        self.data = None

    def callback(self, data):
        data.ranges = list(data.ranges)
        # only detect front 180 degree sector
        for i in range(30, 330):
            data.ranges[i] = 0

        # the closest object
        self.data = min([i for i in data.ranges if i != 0])
        
        forward = True
        if forward and self.data < 0.5:
            print("Obstruct detected")
            self.pub.publish("yes")
        else:
            self.pub.publish("no")

rospy.init_node('obstruct', anonymous=True)
fliter = Fliter()
rospy.spin()