#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from math import sin, cos

class FakeLidar:
    def __init__(self):
        rospy.init_node('fake_lidar')
        self.scan_pub = rospy.Publisher('scan', LaserScan, queue_size=10)
        self.rate = rospy.Rate(10)

    def run(self):
        while not rospy.is_shutdown():
            # Set up the LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = rospy.Time.now()
            scan_msg.header.frame_id = 'laser'
            scan_msg.angle_min = -3.14
            scan_msg.angle_max = 3.14
            scan_msg.angle_increment = 3.14/180
            scan_msg.time_increment = (1.0 / 10) / 360
            scan_msg.range_min = 0.0
            scan_msg.range_max = 10.0

            # Generate fake scan data
            ranges = []
            for i in range(0, 181):
                angle = i * scan_msg.angle_increment
                if angle > -3.14/4 and angle < 3.14/4:
                    ranges.append(5.0)  # obstacle at 5 meters
                else:
                    ranges.append(10.0)  # no obstacle detected
            scan_msg.ranges = ranges

            # Publish the message
            self.scan_pub.publish(scan_msg)

            # Sleep to maintain the desired publishing rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        fake_lidar = FakeLidar()
        fake_lidar.run()
    except rospy.ROSInterruptException:
        pass

