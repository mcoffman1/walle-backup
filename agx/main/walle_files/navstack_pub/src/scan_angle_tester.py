#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(scan_data):
    # Find the index of the closest range in the scan data
    closest_range_index = scan_data.ranges.index(min(scan_data.ranges))
    # Calculate the angle of the closest object
    closest_angle = scan_data.angle_min + closest_range_index * scan_data.angle_increment
    closest_angle_degrees = closest_angle * 180.0 / 3.141592653589793
    rospy.loginfo(f'Closest object at angle: {closest_angle_degrees} degrees')

def listener():
    rospy.init_node('scan_listener', anonymous=True)
    rospy.Subscriber('/walle_scan', LaserScan, scan_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

