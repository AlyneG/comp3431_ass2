#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import time
from std_msgs.msg import Float64

class detectLights:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw/compressed',CompressedImage, self.image_callback)
        self.max_vel_pub = rospy.Publisher('control/max_vel', Float64, queue_size=1)

    def image_callback(self, msg):
        np_arr = numpy.fromstring(msg.data, numpy.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        #have two separate masks
        pink_min = numpy.array([314, 16, 25])
        pink_max = numpy.array([337, 100, 100])
        green_min = numpy.array([112, 21, 25])
        green_max = numpy.array([142, 100, 100])

        #make them into blobs
        mask_pink = cv2.inRange(hsv, pink_min, pink_max)
        mask_green = cv2.inRange(hsv, green_min, green_max)

        cv2.imshow("Pink mask", mask_pink)
        cv2.imshow("Green mask", mask_green)

        #find the centre
        M_pink = cv2.moments(mask_pink)
        M_green = cv2.moments(mask_green)

        cx_pink = None
        cy_pink = None
        cx_green = None
        cy_green = None

        if M_pink['m00'] > 0:
          cx_pink = int(M_pink['m10']/M_pink['m00'])
          cy_pink = int(M_pink['m01']/M_pink['m00'])

        if M_green['m00'] > 0:
          cx_green = int(M_green['m10']/M_green['m00'])
          cy_green = int(M_green['m01']/M_green['m00'])

        #if the coordinates of the moment of one are higher than the other, it is on top
        if cx_pink is not None and cy_pink is not None and cx_green is not None and cy_green is not None:
            print("I see some traffic lights")
            if cy_pink > cy_green: #pink is on the bottom? # possibly add in to check for location in x axis, only work when in certain range?
                #publish a message to go
                print("GO")
                msg_pub_max_vel = Float64()
                msg_pub_max_vel.data = 0.1
                self.max_vel_pub.publish(msg_pub_max_vel)
            elif cy_green > cy_pink: #pink is on top?
                #publish a message to stop
                print("STOP")
                msg_pub_max_vel = Float64()
                msg_pub_max_vel.data = 0
                self.max_vel_pub.publish(msg_pub_max_vel)
        else:
            print("No traffic lights here")

rospy.init_node('detect_lights')
detect_lights = detectLights()
rospy.spin()
