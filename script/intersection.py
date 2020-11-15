#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class intersecionDetection:

    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('camera/image', Image, self.image_callback)
        self.intersection_sub = rospy.Subscriber('control/intersection', Bool, self.intersection_callback) 

        self.detect_intersection_pub = rospy.Publisher('detect/intersection', Bool, queue_size=1)

        self.bool = Bool()

        self.intersection = False

    

    def image_callback(self, msg):

        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        img = image.copy()
        #change perspective
        rows, cols = image.shape[:2]
        rows-=1
        cols-=1

        src_points = numpy.float32([[0,0],[cols, 0],[0,rows], [cols,rows]])
        dst_points = numpy.float32([[0,10],[cols,10], [int(cols*1/7),rows], [int((cols)*6/7),rows]])
        affine_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
        image = cv2.warpPerspective(image, affine_matrix, (cols+1,rows+1))
        image[image==0] = 255
    
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
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
    
        number = numpy.count_nonzero(mask == 255)
        total = h*w*1.0
        prop = number/total
  

        white_lower = numpy.array([0, 0, 0], numpy.uint8) 
        white_upper = numpy.array([0, 0, 255], numpy.uint8) 
        white_mask = cv2.inRange(hsv, white_lower, white_upper)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        # For white color 
        white_mask = cv2.dilate(white_mask, kernel) 

        top = int(0.7*h)
        white_mask[0:top, 0:w] = 0

        res_p = cv2.bitwise_and(img, img, mask = white_mask)

        img1, contours, hierarchy = cv2.findContours(white_mask, 
                                               cv2.RETR_TREE, 
                                               cv2.CHAIN_APPROX_SIMPLE) 
   
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x1, y1, w1, h1 = cv2.boundingRect(contour) 
                img = cv2.rectangle(img, (x1, y1),  
                                    (x1 + w1, y1 + h1),  
                                    (229, 92, 190), 2) 
        
                cv2.putText(img, "White Colour", (x1, y1), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, 
                            (229, 92, 190))

        right = int(0.7*w)
        left = int(0.3*w)
        bottom = int(0.8*h)
        white_mask[:,0:left] = 0
        white_mask[:,right:w] = 0
        white_mask[bottom:h, :] = 0

        number_white = numpy.count_nonzero(white_mask)

        #cv2.imshow("white", white_mask)
        #cv2.waitKey(3)
        #print(number, total, prop)
        if(not self.intersection):
            if(prop <= 0.06):
                #print('FK')
                if(number_white == 0):
                    print('Intersection')
        
                    self.bool.data = True
                    self.detect_intersection_pub.publish(self.bool)
            
                else:

                    self.bool.data = False
                    self.detect_intersection_pub.publish(self.bool)
            else:

                self.bool.data = False
                self.detect_intersection_pub.publish(self.bool)
      
    def intersection_callback(self, msg):

        self.intersection = msg.data

rospy.init_node('intersecionDetection')
intersecionDetection = intersecionDetection()
rospy.spin()
