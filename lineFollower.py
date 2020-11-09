#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
import time

"""
def stop_sign_detection(img_in):
    #Finds the centroid coordinates of a stop sign in the provided
    #image.
    #Args:
    #    img_in (numpy.array BGR): image containing a traffic light.
    #Returns:
    #    (x,y) tuple of the coordinates of the center of the stop sign.
    #    Numpy array: Height x Width matrix of Hough accumulator array  (Height and width from the image)

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
"""

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("window", 1)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                                           Twist, queue_size=1)
        self.twist = Twist()
        self.count = 0
        self.start = False
        self.not_intersection = False

    def image_callback(self, msg):
        if not self.start:
            print("Something is in front of me")
            #self.twist.linear.x = -0.1
            #self.twist.angular.z = 0.1 / 100
            #self.cmd_vel_pub.publish(self.twist)
            return
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #randomly chosen values
        #need to find the lowest and highest threshold for white lines
        #Should be HSV values
        """lower_white = numpy.array([100,  100,  100])
        upper_white = numpy.array([255, 255, 250])"""

        lower_white = numpy.array([0, 0, 212])
        upper_white = numpy.array([131, 255, 255])

        #lower_white = numpy.array([])
        #upper_white = numpy.array([255,255,255])

        #if pixel value is in the range, it sets it to be white (255)
        #else sets it to be black
        mask = cv2.inRange(hsv, lower_white, upper_white)
        v_mask = cv2.inRange(hsv, lower_white, upper_white)
        mask = 255 - mask
        #picks the part that we wanted detect
        #numbers also randomly chosen
        #aims to focus on the bottom part of image
        h, w, d = image.shape
        #search_top = 4*h/5 + 10
        #search_bot = 4*h/5 + 20
        search_top = 4*h/5 + 30
        search_bot = 4*h/5 + 40

        #sets the pixels outside the range to be zero
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        cv2.imshow("mask", mask)


        #function to find the center of the two detected lines??
        #makes red dot in the middle
        #moments() calculates the centroid (srithmetic center) of the blob of the binary image that passes the filter
        """M = cv2.moments(mask)
        if M['m00'] > 0:
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])
          cv2.circle(image, (cx, cy), 20, (0,0,255), -1)         #draws circle on image to help visualisation
          #P-controller: proportional controller
          #linear scaling of an error drives control image_output
          #error = distance between centreline of image and centre of line we are following
          err = cx - w/2                                #calculates error (distance between center col of image & estimated centre of line)
          self.twist.linear.x = 0.08                    #calculate values to be used for cmd_vel stream & scale to Turtlebot
          self.twist.angular.z = -float(err) / 100
          self.cmd_vel_pub.publish(self.twist)          #publishes message
        #print('HAHA')
        #center, h = stop_sign_detection(image)
        #cv2.circle(image, center, 20, (0,255,0), -1)
        #if(center[0] != 0 and center[1] != 0):
         #print('I should stop')
          #time.sleep(5)
          #self.twist.linear.x = 0
          #self.twist.angular.z = 0
          #self.cmd_vel_pub.publish(self.twist)
          cv2.imshow("window", image)
          cv2.waitKey(3)"""


        # find contours in the binary image
        """im2, contours, hierarchy = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)

            # calculate x,y coordinate of center
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0"""

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur1 = cv2.blur(gray,(5,5))
        high_th, th_img = cv2.threshold(blur1,200,255,cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        low_th = 0.5*high_th
        blur = cv2.blur(th_img,(5,5))
        edge = cv2.Canny(blur, low_th, high_th, 30)

        search_top = 3*h/5
        search_bot = h
        edge[0:search_top, 0:w] = 0

        lines = cv2.HoughLines(edge,1,numpy.pi/180, 200,20,0)

        if lines is not None:
            for i in range(len(lines)):
                rho, theta = lines[i][0]
                a = numpy.cos(theta)
                b = numpy.sin(theta)
                x0 = a*rho; y0 = b*rho

                xs = [int(x0 + 1000*(-b)), int(x0 - 1000*(-b))]
                ys = [int(y0 + 1000*(a)), int(y0 - 1000*(a))]
                cv2.line(image,(int(x0 + 1000*(-b)), int(y0 + 1000*(a))),( int(x0 - 1000*(-b)),int(y0 - 1000*(a)) ), (255, 0, 0), 2)

        M = cv2.moments(mask)
        fraction_num = numpy.count_nonzero(mask)
        print(fraction_num)

        if M["m00"] > 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            cv2.putText(image, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            err = cX - w/2                                #calculates error (distance between center col of image & estimated centre of line)

            if self.not_intersection and fraction_num < 2000:
                self.not_intersection = False
                print("I have found an intersection")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(5)
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(3)
            else:
                if fraction_num > 2000:
                    self.not_intersection = True
                print("Following the gap")
                self.twist.linear.x = 0.1                  #calculate values to be used for cmd_vel stream & scale to Turtlebot
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)

        # display the image
        cv2.imshow("Edge", edge)
        cv2.imshow("Image", image)
        cv2.waitKey(3)

    def scan_callback(self, data):
        data.ranges = list(data.ranges)
        # only detect front 180 degree sector
        for i in range(30, 330):
            data.ranges[i] = 0

            # the closest object
        self.data = min([i for i in data.ranges if i != 0])

        self.start = True

        # if something in front of the robot 0.2M, stop moving
        if self.start and self.data < 0.2:
            self.start = False
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            return

rospy.init_node('follower')
follower = Follower()
rospy.spin()
