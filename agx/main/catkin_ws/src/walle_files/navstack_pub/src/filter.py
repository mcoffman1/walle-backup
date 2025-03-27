#!/usr/bin/env python
 
import rospy
from sensor_msgs.msg import LaserScan

class ScanFilter:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.filter_pub = rospy.Publisher('/walle_scan', LaserScan, queue_size=10)
        self.filter_msg = LaserScan()
        
        self.minone = 320
        self.maxone = 860

    def scan_callback(self, msg):
        self.filter_msg.ranges = []
        self.filter_scan(msg)

    def filter_scan(self, msg):
        for i,r in enumerate(msg.ranges):
            if i >= (self.minone) and i <= (self.maxone):
                self.filter_msg.ranges.append(float('inf'))
            else:
                self.filter_msg.ranges.append(r)

        self.filter_msg.header = msg.header
        self.filter_msg.angle_min = msg.angle_min
        self.filter_msg.angle_max = msg.angle_max
        self.filter_msg.angle_increment = msg.angle_increment
        self.filter_msg.range_min = msg.range_min
        self.filter_msg.range_max = msg.range_max
        self.filter_msg.intensities = msg.intensities
        #self.filter_msg.ranges = 
        self.filter_pub.publish(self.filter_msg)



if __name__ == '__main__':
    rospy.init_node('scan_filter', anonymous=True)
    sf = ScanFilter()
    rospy.spin() # Keeps code from exiting until the node is stopped

