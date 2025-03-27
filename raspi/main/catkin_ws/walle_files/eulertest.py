#!/usr/bin/env python

import tf.transformations as tr
from nav_msgs.msg import Odometry

odomOld = Odometry()
x = odomOld.pose.pose.orientation.x
y = odomOld.pose.pose.orientation.y
z = odomOld.pose.pose.orientation.z
w = odomOld.pose.pose.orientation.w

quat = (x,y,z,w)
roll, pitch, yaw = tr.euler_from_quaternion(quat)

print(yaw)
