#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

def publish_once():
    # Initialize the ROS node
    rospy.init_node('head_pos_publisher', anonymous=True)
    
    # Create a publisher for the 'head_pos' topic
    pub = rospy.Publisher('head_pos', Int16, queue_size=10)
    
    # Wait for the publisher to connect to subscribers
    rospy.sleep(1)
    
    # Create an Int16 message with data = 2
    msg = Int16()
    msg.data = 2
    
    # Publish the message once
    pub.publish(msg)
    
    # Log the published message
    rospy.loginfo(f"Published {msg.data} to head_pos")

if __name__ == '__main__':
    try:
        publish_once()
    except rospy.ROSInterruptException:
        pass

