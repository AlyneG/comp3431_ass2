import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Fliter:
    def __init__(self):
        self.sub = rospy.Subscriber('scan', LaserScan, self.callback)
        # self.pub = rospy.Publisher('filteredscan', LaserScan, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.data = None

    def callback(self, data):
        data.ranges = list(data.ranges)
        # only detect front 180 degree sector
        for i in range(30, 330):
            data.ranges[i] = 0

        # the closest object
        self.data = min([i for i in data.ranges if i != 0])
        
        forward = True
        if forward and self.data < 0.3:
            forward = False
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            return
        self.twist.linear.x = 0.15
        self.cmd_vel_pub.publish(self.twist)

rospy.init_node('laser_listener', anonymous=True)
fliter = Fliter()
rospy.spin()