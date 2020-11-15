#!/usr/bin/env python
import rospy, numpy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class objectDetection:

    def __init__(self):

        self.scan_sub = rospy.Subsriber('scan', LaserScan, self.scan_callback)

        self.detect_object_pub = rospy.Publisher('detect/object', Bool, queue_size=1)

        self.bool = Bool()


    def scan_callback(self, data):

        data.ranges = list(data.ranges)

        # only detect front 60 degree sector
        for i in range(30, 330):
            data.ranges[i] = 0

        # the closest object
        self.data = min([i for i in data.ranges if i != 0])

        # if someting in front of the robot 0.2M, stop moving
        if self.data < 0.2:
            # There is an object ahead
            self.bool.data = True
            self.detect_object_pub.publish(self.bool)

        else:
            # No object ahead
            self.bool.data = False
            self.detect_object_pub.publish(self.bool)


rospy.init_node('objectDetection')
objectDetection = objectDetection()
rospy.spin()
