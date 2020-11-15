#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64


class Control:

    def __init__(self):

        self.intersection_sub = rospy.Subscriber('detect/intersection', Bool, self.intersection_callback)
        self.stopSign_sub = rospy.Subscriber('detect/stopSign', Bool, self.stopSign_callback)
        self.color_sub = rospy.Subscriber('detect/color', Float64, self.color_callback)
        self.object_sub = rospy.Subscriber('detect/object', Bool, self.object_callback)
        self.move_sub = rospy.Subscriber('control/move', Bool, self.move_callback)

        self.max_vel_pub = rospy.Publisher('control/max_vel', Float64, queue_size=1)
        self.stop_pub = rospy.Publisher('control/stop', Bool, queue_size=1)
        self.intersection_pub = rospy.Publisher('control/intersection', Bool, queue_size=1)

        self.intersection = False
        self.stopSign = False
        self.color_code = -1
        self.object = False

        self.max_vel = Float64()
        self.stop = Bool()

    def intersection_callback(self, msg):

        self.intersection = msg.data

        if(self.intersection):
            #print(self.stopSign)
            self.stop.data = True
            self.intersection_pub.publish(self.stop)
            
            if(self.stopSign):
                self.max_vel.data = 0
                self.max_vel_pub.publish(self.max_vel)

            elif(self.color_code != -1):
                if(self.color_code == 0):
                    # yellow
                    self.max_vel.data = 0.05
                    self.max_vel_pub.publish(self.max_vel)
                elif(self.color_code == 1):
                    # green
                    self.max_vel.data = 0.1
                    self.max_vel_pub.publish(self.max_vel)
                else:
                    # red
                    self.max_vel.data = 0
                    self.max_vel_pub.publish(self.max_vel)

                    self.stop.data = True
                    self.stop_pub.publish(self.stop)

            else:

                #print('6')
                self.max_vel.data = 0
                self.max_vel_pub.publish(self.max_vel)

                self.stop.data = True
                self.stop_pub.publish(self.stop)


        
    def stopSign_callback(self, msg):
        self.stopSign = msg.data
        if(not self.stopSign and not self.intersection):
            #print('NO')
            self.max_vel.data = 0.1
            self.max_vel_pub.publish(self.max_vel)

        
    
    def color_callback(self, msg):
        
        self.color_code = msg.data

        if(self.color_code == -1):
            self.max_vel = -1
            self.max_vel_pub.publish(self.max_vel)

    def object_callback(self, msg):

        self.object = msg.data

        if(self.object):
            self.max_vel.data = 0
            self.max_vel_pub.publish(self.max_vel)
        else:
            self.max_vel.data = 0.1
            self.max_vel_pub.publish(self.max_vel)

    def move_callback(self, msg):

        move = msg.data
        print(move)
        if(move):
            self.max_vel.data = 0.1
            self.max_vel_pub.publish(self.max_vel)
        
rospy.init_node('control')
control = Control()
rospy.spin()
