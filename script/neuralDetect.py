#!/usr/bin/env python
import rospy, cv_bridge, numpy
import cv2
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from tensorflow.keras.models import load_model
import time
import os
start = True
lower_real = [0,110,0]
lower_gazebo = [0,0,0]
class Intersection:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/image', Image, self.intersection_detect)
        self.pub = rospy.Publisher('intersection',String,queue_size=10)
        print(os.getcwd())
        self.model = load_model('/home/rsa/catkin_ws/src/comp3431_ass2/script/model/model.h5')

    def intersection_detect(self, image):
        image = self.bridge.imgmsg_to_cv2(image,desired_encoding='bgr8')
        #change perspective
        rows, cols = image.shape[:2]
        rows-=1
        cols-=1

        #src_points = numpy.float32([[int(cols*1.5/4),int(rows*4/7)],[int(cols*2.5/4), int(rows*4/7)],[0,rows], [cols,rows]])
        src_points = numpy.float32([[0,0],[cols, 0],[0,rows], [cols,rows]])
        dst_points = numpy.float32([[0,10],[cols,10], [int(cols*1/7),rows], [int((cols)*6/7),rows]])
        affine_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        image = cv2.warpPerspective(image, affine_matrix, (cols+1,rows+1))
        image[image==0] = 255
        intersection = False
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _,mask = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
        h, w, d = image.shape
        search_top = int(8.5*h/10)
        search_bot = int(h) 
        it = 40
        dilate = cv2.dilate(mask,None, iterations=it)
        mask = cv2.erode(dilate,None, iterations=it)
        mask = 255 - mask
        mask[search_bot:h, 0:w] = 0
        mask[0:search_top, 0:w] = 0
        #print(number,total,prop)
        #cv2.imshow("stop",mask)
        a = self.model.predict(numpy.array([mask[350:,50:600]/255.0]))
        print(a)
        cv2.imshow("window", mask)
        if(a[0][0] <= a[0][1]):
            print("Found intersection")
            #self.pub.publish("yes")
            time.sleep(3)
            #self.pub.publish("no")
            #time.sleep(20)
        else:
            self.pub.publish("no")
        #cv2.imshow("window", mask)
        cv2.waitKey(3)


rospy.init_node('intersection')
inters = Intersection()
rospy.spin()
