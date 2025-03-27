#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
import time

def publish_initial_pose():
    starttime = time.time()
    rospy.init_node('initial_pose_publisher', anonymous=True)
    pub = rospy.Publisher('initial_2d', PoseStamped, latch=True, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    # Set the initial pose values
    initial_pose = PoseStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    # Publish the initial pose
    while not rospy.is_shutdown():
        pub.publish(initial_pose)
        rate.sleep()
        if time.time() - starttime >= 5:
            break

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
