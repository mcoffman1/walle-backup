#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Calculate the indices for the desired scan range
    angle_min = msg.angle_min
    angle_max = msg.angle_max
    angle_increment = msg.angle_increment

    # Define the desired scan range in radians
    desired_min_angle = -3.14159   # -180 degrees
    desired_max_angle = -0.5 * 3.14159    # -90 degrees
    desired_min_angle2 = 0.5 * 3.14159    # 90 degrees
    desired_max_angle2 = 3.14159    # 180 degrees

    # Calculate the corresponding indices in the scan data array
    desired_min_index = int((desired_min_angle - angle_min) / angle_increment)
    desired_max_index = int((desired_max_angle - angle_min) / angle_increment)
    desired_min_index2 = int((desired_min_angle2 - angle_min) / angle_increment)
    desired_max_index2 = int((desired_max_angle2 - angle_min) / angle_increment)

    # Filter the scan data
    filtered_ranges = msg.ranges[:desired_min_index] + msg.ranges[desired_max_index+1:desired_min_index2] + msg.ranges[desired_max_index2+1:]

    # Create a new LaserScan message with the filtered data
    filtered_msg = LaserScan()
    filtered_msg.header = msg.header
    filtered_msg.angle_min = angle_min
    filtered_msg.angle_max = angle_max
    filtered_msg.angle_increment = angle_increment
    filtered_msg.range_min = msg.range_min
    filtered_msg.range_max = msg.range_max
    filtered_msg.ranges = filtered_ranges

    # Publish the filtered message on the /walle_scan topic
    filtered_pub.publish(filtered_msg)

if __name__ == '__main__':
    rospy.init_node('scan_filter_node')

    # Create a publisher for the filtered scan data
    filtered_pub = rospy.Publisher('/walle_scan', LaserScan, queue_size=10)

    # Subscribe to the original scan data
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.spin()

