#!/usr/bin/env python3

import rospy
import paho.mqtt.client as mqtt
from vision_msgs.msg import Detection2DArray  # Make sure this is the correct message type

class PeopleDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('people_detector')

        # Initialize MQTT client
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("10.0.0.34", 1883, 60)  # Replace with your MQTT broker's address

        # Subscribe to the detectnet/detections topic
        self.subscriber = rospy.Subscriber('detectnet/detections', Detection2DArray, self.callback)

        # Keep track of the last number of detected people
        self.last_num_people = 0
        self.latest_msg = None
        self.timer = rospy.Timer(rospy.Duration(1), self.process_msg)  # Process every 1 second, adjust as needed

    def callback(self, msg):
        self.latest_msg = msg  # Store the latest message

    def process_msg(self, event):
        if self.latest_msg:
            # Parse the message to count the number of detected people
            num_people = sum(1 for detection in self.latest_msg.detections if detection.results[0].id == 1)
            # Publish the number of people to the MQTT broker
            self.mqtt_client.publish("people_count", num_people)
            #print(num_people)
            self.latest_msg = None  # Reset the latest message
        else:
            # Parse the message 0 because no people have been detected
            num_people = 0
            # Publish the number of people to the MQTT broker
            self.mqtt_client.publish("people_count", num_people)
            #print(num_people)
            self.latest_msg = None  # Reset the latest message
            

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    detector = PeopleDetector()
    detector.spin()

