#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time
import os
start = True
lower_real = [0,110,0]
lower_gazebo = [0,0,0]

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.numImage = 0
    self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
    self.inter_sub = rospy.Subscriber('intersection', String, self.inter_callback)    
    self.stop_sub = rospy.Subscriber('stop', String, self.stop_callback)
    self.turn_sub = rospy.Subscriber('turn', String, self.turn_callback)      
    self.obstruct_sub = rospy.Subscriber('obstruct', String, self.obstruct_callback)
    self.inter = False
    self.stop = False
    self.obstruct = False
    self.turn = None
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):

    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    #change perspective
    rows, cols = image.shape[:2]
    rows-=1
    cols-=1

    src_points = numpy.float32([[0,0],[cols, 0],[0,rows], [cols,rows]])
    dst_points = numpy.float32([[0,10],[cols,10], [int(cols*1/7),rows], [int((cols)*6/7),rows]])
    affine_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
    image = cv2.warpPerspective(image, affine_matrix, (cols+1,rows+1))
    image[image==0] = 255
    #crop the detect position
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _,mask = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
    h, w, d = image.shape
    search_top = int(8.5*h/10)
    search_bot = int(h)
    #dilate and erode to make the line be more clear
    it = 20
    dilate = cv2.dilate(mask,None, iterations=it)
    mask = cv2.erode(dilate,None, iterations=it)
    mask = 255 - mask
    mask[search_bot:h, 0:w] = 0
    mask[0:search_top, 0:w] = 0
    M = cv2.moments(mask)

    cv2.imshow("mask",image)
    #if a turn signal is detected, turn left or right
    if(self.turn):
      print("turn signal")
      self.sendMessage(self.turn)
      return
    #if a intersection of stop sing is detect, stop
    ru = 1
    if(self.inter or self.stop or self.obstruct):
      ru = 0
    #print("send message",ru)

    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 4, (0,0,255), -1)
      err = cx - w/2
      self.twist.linear.x = 0.1 * ru
      self.twist.angular.z = -float(err) / 40 * ru
      self.cmd_vel_pub.publish(self.twist)

    cv2.waitKey(3)

  def sendMessage(self,left):
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


  def inter_callback(self,data):
    self.inter = data.data =='yes'

  def stop_callback(self,data):
    self.stop = data.data =='yes'

  def turn_callback(self,data):
    if(data.data == "left"):
      self.turn = True
    elif(data.data == "right"):
      self.turn = False
    else:
      self.turn = None

  def obstruct_callback(self,data):
    self.obstruct = data.data == "yes"

rospy.init_node('follower')
follower = Follower()
rospy.spin()
