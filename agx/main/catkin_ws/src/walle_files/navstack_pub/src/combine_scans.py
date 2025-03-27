#!/usr/bin/env python
 
import rospy
from sensor_msgs.msg import LaserScan

class ScanFilter:
    def __init__(self):
        rospy.init_node('scan_filter', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        self.filter_pub1 = rospy.Publisher('/scan_left', LaserScan, queue_size=10)
        self.filter_pub2 = rospy.Publisher('/scan_right', LaserScan, queue_size=10)
        self.combined_pub = rospy.Publisher('/walle_scan', LaserScan, queue_size=10)
        self.filter_msg1 = LaserScan()
        self.filter_msg2 = LaserScan()
        
        self.minone = 0
        self.maxone = 286
        self.mintwo = 860
        self.maxtwo = 1146

    def scan_callback(self, msg):
        self.filter_msg1.ranges = []
        self.filter_msg2.ranges = []
        self.filter_msg1.intensities = []
        self.filter_msg2.intensities = []
        self.filter_scan_left(msg)
        self.filter_scan_right(msg)
        self.combine_scans()

    def filter_scan_left(self, msg):
        for i,inten in enumerate(msg.intensities):
            if i >= (self.minone) and i <= (self.maxone): self.filter_msg1.intensities.append(inten)
        for i,r in enumerate(msg.ranges):
            if i >= (self.minone) and i <= (self.maxone): self.filter_msg1.ranges.append(r)

        self.filter_msg1.header = msg.header
        self.filter_msg1.angle_min = msg.angle_min
        self.filter_msg1.angle_max = self.maxone * msg.angle_increment
        self.filter_msg1.angle_increment = msg.angle_increment
        self.filter_msg1.range_min = msg.range_min
        self.filter_msg1.range_max = msg.range_max
        #self.filter_msg1.ranges = 
        self.filter_pub1.publish(self.filter_msg1)
        
    def filter_scan_right(self, msg):
        for i,inten in enumerate(msg.intensities):
            if i >= (self.mintwo) and i <= (self.maxtwo): self.filter_msg2.intensities.append(inten)
        for i,r in enumerate(msg.ranges):
            if i >= (self.mintwo) and i <= (self.maxtwo): self.filter_msg2.ranges.append(r)

        self.filter_msg2.header = msg.header
        self.filter_msg2.angle_min = self.maxone * msg.angle_increment
        self.filter_msg2.angle_max = self.filter_msg2.angle_min + (self.maxone * msg.angle_increment)
        self.filter_msg2.angle_increment = msg.angle_increment
        self.filter_msg2.range_min = msg.range_min
        self.filter_msg2.range_max = msg.range_max
        #self.filter_msg1.ranges = 
        self.filter_pub2.publish(self.filter_msg2)
        
    def combine_scans(self):
        combined_scan = LaserScan()
        # Set the header and other metadata
        combined_scan.header = self.filter_msg1.header
        combined_scan.angle_min = self.filter_msg1.angle_min
        combined_scan.angle_max = self.filter_msg2.angle_max
        combined_scan.angle_increment = self.filter_msg1.angle_increment
        combined_scan.range_min = self.filter_msg1.range_min
        combined_scan.range_max = self.filter_msg1.range_max

        # Combine the ranges and intensities
        combined_scan.ranges = self.filter_msg1.ranges + [0] * (self.mintwo - self.maxone) + self.filter_msg2.ranges
        combined_scan.intensities = self.filter_msg1.intensities + [0] * (self.mintwo - self.maxone) + self.filter_msg2.intensities

        # Publish the combined scan
        self.combined_pub.publish(combined_scan)



if __name__ == '__main__':
    sf = ScanFilter()
    rospy.spin() # Keeps code from exiting until the node is stopped

