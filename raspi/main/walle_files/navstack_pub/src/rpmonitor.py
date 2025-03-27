#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Log

def callback(msg):
    # Filtering messages from the rplidarNode
    if msg.name in ["/rplidarNode", "/launcher"]:
        rospy.loginfo("[{}]: {}".format(msg.level, msg.msg))

rospy.init_node('monitor')
rospy.Subscriber('/rosout', Log, callback)
rospy.spin()

