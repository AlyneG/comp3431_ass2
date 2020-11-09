#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float64
from std_msgs.msg import Bool

class Intersection:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel',Twist, queue_size=1)
    self.max_vel_pub = rospy.Publisher('control/max_vel', Float64, queue_size=1)
    self.intersection_detection_pub = rospy.Publisher('detect/intersection', Bool, queue_size=1)
    self.twist = Twist()
    self.bool = Bool()
    #self.count = 0

  def image_callback(self, msg):

    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([100,100,100])
    upper_yellow = numpy.array([150,255,255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #res = cv2.bitwise_and(image, image, mask = mask)


    h, w, d = hsv.shape
    #print(h, w, d)
    #search_top = int(4*h/5+10)
    #search_bot = int(h)
    #mask[0:search_top, 0:w] = 255
    #mask[search_bot:h, 0:w] = 255
    #cv2.HoughLinesP(img, 1, np.pi/180, 10, minLineLength=50, maxLineGap=30)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
 
    #gaus = cv2.GaussianBlur(gray,(3,3),0)
 
    edges = cv2.Canny(mask, 100, 200, apertureSize=3)
    #cv2.circle(image, (0,0), 10, (255,0,0),-1)
    #cv2.circle(image, (0,479), 10, (255,0,0),-1)
    #cv2.circle(image, (639,0), 10, (255,0,0),-1)
    #edges[0:int(3*h/5), 0:w] = 0
    lines = cv2.HoughLinesP(edges, rho=1, theta=numpy.pi/180, threshold=100, minLineLength=60, maxLineGap=50)
    #cv2.HoughLinesP(mask, rho=6, theta=numpy.pi / 180, threshold=200, lines=numpy.array([]),minLineLength=50,maxLineGap=0)
    if(lines is not None):
        for x1,y1,x2,y2 in lines[0]:
            if(y1 >= int(5*h/6) and y2 >= int(5*h/6)):
                print('Intersection detected')
                cv2.line(image,(x1,y1),(x2,y2),(0,255,0),3)
                self.bool.data = True
                self.intersection_detection_pub.publish(self.bool)
                #rospy.sleep(5)
                #self.bool.data = False
                #self.intersection_detection_pub.publish(self.bool)
                #mid_x = int((x1+x2)/2)
                #mid_y = int((y1+y2)/2)
                #cv2.circle(image, (mid_x, mid_y), 10, (0,255,0),-1)
                #print(mid_x, mid_y - 100)
                #if(mid_x < w and mid_y < h):
                #cv2.circle(image, (mid_x, mid_y-5), 10, (255,0,0),-1)
                #cv2.circle(image, (mid_x,h-3), 10, (255,0,0),-1)
                #if(image[:,:,0][h-1,mid_x] == image[:,:,0][mid_y-100,mid_x] and image[:,:,1][h-1,mid_x] == image[:,:,1][mid_y-100,mid_x] and image[:,:,2][h-1,mid_x] == image[:,:,2][mid_y-100,mid_x]):
                #if(hsv[:,:,0][h-1, mid_x] == hsv[:,:,0][mid_y-100, mid_x]):
                #    print('FK I NEED TO STOP')
                    #print(mid_x, mid_y-100)
                    #cv2.circle(image, (x1,y1+80), 10, (255,0,0),-1)

    #print(image.shape)
    cv2.imshow("window", image)
    cv2.waitKey(3)
    '''
    if(image[:,:,0][int(w/2), int(5*h/6 + 20)] == 255 and image[:,:,1][int(w/2), int(5*h/6 + 20)] == 255 and image[:,:,2][int(w/2), int(5*h/6 + 20)] == 255
        and image[:,:,0][int(w/2-100), int(5*h/6 + 20)] == 255 and image[:,:,1][int(w/2-100), int(5*h/6 + 20)] == 255 and image[:,:,2][int(w/2-100), int(5*h/6 + 20)] == 255):
        if(image[:,:,0][int(w/2), int(5*h/6-60)] == image[:,:,0][int(w/2), int(5*h/6 + 80)] and image[:,:,1][int(w/2), int(5*h/6-60)] == image[:,:,1][int(w/2), int(5*h/6 + 80)] and image[:,:,2][int(w/2), int(5*h/6-60)] == image[:,:,2][int(w/2), int(5*h/6 + 80)]):
            print(image[:,:,0][int(w/2), int(5*h/6 + 20)], image[:,:,1][int(w/2), int(5*h/6 + 20)], image[:,:,2][int(w/2), int(5*h/6 + 20)])
            print(image[:,:,0][int(w/2-100), int(5*h/6 + 20)], image[:,:,1][int(w/2-100), int(5*h/6 + 20)], image[:,:,2][int(w/2-100), int(5*h/6 + 20)])
            #cv2.circle(image, (w/2, 5*h/6 + 20), 10, (255,0,0),-1)
            #cv2.circle(image, (w/2-100, 5*h/6 + 20), 10, (0,0,255),-1)
            print('HAHAHA')
            if(lines is not None):
                for x1,y1,x2,y2 in lines[0]:
                    if(y1 >= int(5*h/6 + 20) and y2 >= int(5*h/6 + 20)):
                        print('STOP')
                        msg_pub_max_vel = Float64()
                        msg_pub_max_vel.data = 0
                        self.max_vel_pub.publish(msg_pub_max_vel)

                        rospy.sleep(5)
                        print('MOVE')
                        msg_pub_max_vel = Float64()
                        msg_pub_max_vel.data = 0.3
                        self.max_vel_pub.publish(msg_pub_max_vel)
                        cv2.line(image,(x1,y1),(x2,y2),(0,255,0),3)
    '''              
       
    #cv2.circle(image, (w/2, 5*h/6 + 20), 10, (255,0,0),-1)
    #cv2.circle(image, (w/2-100, 5*h/6 + 20), 10, (0,0,255),-1)
    #cv2.circle(image, (int(w/2), int(5*h/6-60)), 10, (0,255,0),-1)
    #cv2.circle(image, (int(w/2), int(5*h/6 + 80)), 10, (0,255,0),-1)
    #M = cv2.moments(ma
    #  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    #  err = cx - w/2
    #  self.twist.linear.x = 0.08
    #  self.twist.angular.z = -float(err) / 100
      #print(-float(err) / 60)
    #  self.cmd_vel_pub.publish(self.twist)
    #print('HAHA')
    #center, h = stop_sign_detection(image)
    #cv2.circle(image, center, 20, (0,255,0), -1)
    #if(center[0] != 0 and center[1] != 0):
    #  print('I should stop')
      #time.sleep(5)
      #self.twist.linear.x = 0
      #self.twist.angular.z = 0
      #self.cmd_vel_pub.publish(self.twist)
    #cv2.imshow("window", image)
    #cv2.waitKey(3)


    #if M['m00'] > 0:
    #  image cx = int(M['m10']/M['m00'])
    #  cy = int(M['m01']/M['m00mask'])
    #  cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
    #  err = cx - w/2
    #  self.twist.linear.x = 0.08
    #  self.twist.angular.z = -float(err) / 100
      #print(-float(err) / 60)
    #  self.cmd_vel_pub.publish(self.twist)
    #print('HAHA')
    #center, h = stop_sign_detection(image)
    #cv2.circle(image, center, 20, (0,255,0), -1)
    #if(center[0] != 0 and center[1] != 0):
    #  print('I should stop')
      #time.sleep(5)
      #self.twist.linear.x = 0
      #self.twist.angular.z = 0
      #self.cmd_vel_pub.publish(self.twist)
    #cv2.imshow("window", image)
    #cv2.waitKey(3)

rospy.init_node('intersection')
follower = Intersection()
rospy.spin()
