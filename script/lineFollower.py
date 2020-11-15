#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class Follower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.move_sub = rospy.Subscriber('control/max_vel', Float64, self.max_vel_callback)   

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.twist = Twist()

        self.max_vel = 0.1

        self.counter = 1

    
    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #change perspective
        rows, cols = image.shape[:2]
        rows-=1
        cols-=1

        #lower_white = numpy.array([0,0,0])  
        #upper_white = numpy.array([0,0,255])
        lower_white = numpy.array([80,100,100])
        upper_white = numpy.array([200,255,250])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        #src_points = numpy.float32([[0,0],[cols, 0],[0,rows], [cols,rows]])
        #dst_points = numpy.float32([[0,10],[cols,10], [int(cols*1/7),rows], [int((cols)*6/7),rows]])
        #affine_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        #image = cv2.warpPerspective(image, affine_matrix, (cols+1,rows+1))
        #image[image==0] = 255

  
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        #_,mask = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
        h, w, d = image.shape
        search_top = int(0.8*h)
        search_bot = int(h) 
        #it = 40
        #dilate = cv2.dilate(mask,None, iterations=it)
        #mask = cv2.erode(dilate,None, iterations=it)
        #mask = 255 - mask
        #left = int(0.2*w)
        #right = int(0.8*w)
        mask[search_bot:h, 0:w] = 0
        mask[0:search_top, 0:w] = 0
        #mask[:, 0:left] = 0
        #mask[:, right:w] = 0

        cv2.imshow("mask", mask)
        cv2.waitKey(3)
      
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 4, (0,0,255), -1)
            err = cx - w/2
            self.twist.linear.x = self.max_vel
            self.twist.angular.z = -(float(err)) / 70
            self.cmd_vel_pub.publish(self.twist)


        cv2.imshow("lineFollower", image)
        cv2.waitKey(3)

    def max_vel_callback(self, msg):

        self.max_vel = msg.data

rospy.init_node('follower')
follower = Follower()
rospy.spin()
