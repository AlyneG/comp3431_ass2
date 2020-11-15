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
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    self.twist = Twist()
    self.move = True
    self.countnointer = 0
    self.countinter = 0

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
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
    '''
    cv2.circle(image, (0, rows), 5, (0,0,255), -1)
    cv2.circle(image, (cols, rows), 5, (0,0,255), -1)
    cv2.circle(image, (int(cols*1.3/4), int(rows*4/7)), 5, (0,0,255), -1)
    cv2.circle(image, (int(cols*2.7/4), int(rows*4/7)), 5, (0,0,255), -1)
    '''

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _,mask = cv2.threshold(gray,150,255,cv2.THRESH_BINARY)
    h, w, d = image.shape
    search_top = int(8.5*h/10)
    search_bot = int(h) 
    it = 30
    dilate = cv2.dilate(mask,None, iterations=it)
    mask = cv2.erode(dilate,None, iterations=it)
    mask = 255 - mask
    mask[search_bot:h, 0:w] = 0
    mask[0:search_top, 0:w] = 0
    M = cv2.moments(mask)
    image[:,:,]

    #print(M)
    ru = 1
    if not self.move:
      ru = 0
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 4, (0,0,255), -1)
      err = cx - w/2
      self.twist.linear.x = 0.1 * ru
      self.twist.angular.z = -float(err) / 40 * ru
      self.cmd_vel_pub.publish(self.twist)
    '''if(ru == 1):
      os.chdir("/home/rsa/image/intersection/isInter")
      cv2.imwrite(str(self.countnointer)+".jpg", mask)
      self.countnointer += 1
    else:
      os.chdir("/home/rsa/image/intersection/notInter")
      cv2.imwrite(str(self.countinter)+".jpg", mask)
      self.countinter+=1'''
    cv2.waitKey(3)

  def inter_callback(self,data):
    self.move = data.data=='no'

rospy.init_node('follower')
follower = Follower()
rospy.spin()
