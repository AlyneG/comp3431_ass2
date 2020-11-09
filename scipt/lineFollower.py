#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time

def stop_sign_detection(img_in):
    """Finds the centroid coordinates of a stop sign in the provided
    image.
    Args:
        img_in (numpy.array BGR): image containing a traffic light.
    Returns:
        (x,y) tuple of the coordinates of the center of the stop sign.
        Numpy array: Height x Width matrix of Hough accumulator array  (Height and width from the image)
    """
    #raise NotImplementedError
    row = img_in.shape[0]
    col = img_in.shape[1]
    H = numpy.zeros((row, col, 21))
    gray = cv2.cvtColor(img_in, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img_in, cv2.COLOR_BGR2HSV)
    edges = cv2.Canny(gray, 80, 100)#feature.canny(gray, sigma=3, low_threshold=50, high_threshold=100)#cv2.Canny(gray,100,200)#cv2.Canny(gray,50,100)#feature.canny(gray, sigma=3)#cv2.Canny(gray,100,200)
    #plt.pyplot.imsave('../a.jpg', edges)

    h, w, d = img_in.shape
    search_top = int(1*h/6 + 70)
    search_bot = int(3*h/6 - 40)
    edges[0:search_top, 0:w] = 0
    edges[search_bot:h, 0:w] = 0


    edges_indices = numpy.where(edges[:,:] > 0)
    if(len(edges_indices) < 2):
      return (0,0), 0
    indices_x = edges_indices[0]
    indices_y = edges_indices[1]
    if(len(indices_x) == 0 or len(indices_y) == 0):
      return (0,0), 0
    #indices_x.sort()
    #indices_y.sort()
    max_x = max(indices_x)
    min_x = min(indices_x)
    max_y = max(indices_y)
    min_y = min(indices_y)
    #print(indices_x)
    #print(indices_y)
    num_points = edges_indices[0].shape[0]
    theta = numpy.arange(0,360)*numpy.pi/180.0
    #print(edges_indices)
    for edge_point in range(num_points):
        x = edges_indices[0][edge_point]
        y = edges_indices[1][edge_point]
        if((x == min_x or x == max_x) or (y == min_y or y == max_y)):
            #img_in[x, y] = [255, 0, 0]
            for r in range(50, 51):
                for angle in theta:
                    a = int(numpy.floor(x - r*numpy.cos(angle)))
                    b = int(numpy.floor(y + r*numpy.sin(angle)))
                    if(a < H.shape[0] and b < H.shape[1]):
                        H[a, b, r-40] = H[a, b, r-40] + 1
                    #print(H[a, b, r-20])

    max_value = numpy.amax(H)
    flat=H.flatten()
    flat.sort()
    #print(flat)
    biggest = flat[-1]
    indices = numpy.argwhere(H == biggest)

    center_a = 0
    center_b = 0
    if(len(indices) > 0):
      radius = indices[0][2]
      center_a = indices[0][1]
      center_b = indices[0][0]

    #print(radius)
    #plt.pyplot.imsave('../c.jpg', img_in)
    #print(center_a, center_b)
    return (center_a, center_b), H[:, :, radius]

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()
    self.count = 0

  def image_callback(self, msg):

    print("HI")
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #randomly chosen values
    lower_yellow = numpy.array([0,  0,  0])
    upper_yellow = numpy.array([255, 255, 250])
    #need to find the lowest and highest threshold for white lines
    #opencv rgb value

    #if pixel value is in the range, it sets it to be white (255)
    #else sets it to be black
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    #picks the part that we wanted detect
    #numbers also randomly chosen
    #aims to focus on the bottom part of image
    h, w, d = image.shape
    search_top = 4*h/5 + 10
    search_bot = 4*h/5 + 20
    
    #sets the pixels outside the range to be zero
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0

    #function to find the center of the two detected lines??
    #makes red dot in the middle
    M = cv2.moments(mask)
    print(M['m00'])
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      err = cx - w/2
      self.twist.linear.x = 0.08
      self.twist.angular.z = -float(err) / 100
      #print(-float(err) / 60)
      print("Hello")
      self.cmd_vel_pub.publish(self.twist)
      print("I completed the publish?")
    #print('HAHA')
    #center, h = stop_sign_detection(image)
    #cv2.circle(image, center, 20, (0,255,0), -1)
    #if(center[0] != 0 and center[1] != 0):
     #print('I should stop')
      #time.sleep(5)
      #self.twist.linear.x = 0
      #self.twist.angular.z = 0
      #self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("window", mask)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
