#!/usr/bin/env python

import rospy, cv2, numpy
from std_msgs.msg import Bool


class timeControl:

    def __init__(self):
        
        self.stop_sub = rospy.Subscriber('control/stop', Bool, self.stop_callback)
        
        self.move_pub = rospy.Publisher('control/move', Bool, queue_size=1)

    def stop_callback(self, msg):

        stop = msg.data

        if(stop):
            
            rospy.sleep(5)

            msg_move = Bool()
            msg_move.data = True
            self.move_pub.publish(msg_move)

rospy.init_node('timeControl')
timeControl = timeControl()
rospy.spin()