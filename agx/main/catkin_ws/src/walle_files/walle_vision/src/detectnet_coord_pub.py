import rospy
from vision_msgs.msg import Detection2DArray  # Import the correct message type
from std_msgs.msg import Int16

class DetectionPubSub():
    def __init__(self) -> None:
        rospy.init_node('detection_pubsub', anonymous=True)
        self.sub = rospy.Subscriber("/detectnet/detections", Detection2DArray, self.callback)
        
        self.pospub = rospy.Publisher('head_pos', Int16, queue_size=10)
        self.posxpub = rospy.Publisher('head_x', Int16, queue_size=10)
        self.posypub = rospy.Publisher('head_y', Int16, queue_size=10)

        self.head_x = 95
        self.head_y = 15

        self.last_time = rospy.get_time()
        self.current_time = rospy.get_time()
        
    def callback(self, data):
        self.current_time = rospy.get_time()
        if self.current_time - self.last_time > 0.2:  # Process detections every 0.2 seconds
            for detection in data.detections:
                for result in detection.results:
                    if result.id == 1:  # Check if the detected ID is 1
                        # Determine x based on the conditions
                        if detection.bbox.center.x < 220:
                            x_val = 2
                        elif detection.bbox.center.x < 500:
                            x_val = 1
                        elif detection.bbox.center.x > 1000:
                            x_val = -2
                        elif detection.bbox.center.x > 780:
                            x_val = -1
                        else:
                            x_val = 0  # For cases not covered in the conditions

                        # Calculate the top of the bounding box
                        top_y = detection.bbox.center.y - (detection.bbox.size_y / 2)

                        # Determine y based on the conditions, using the top of the box
                        if top_y > 400:
                            y_val = 1
                        elif top_y < 100:
                            y_val = -1
                        else:
                            y_val = 0  # For cases not covered in the conditions

                        # Update head_x with bounds checking
                        self.head_x = min(max(self.head_x + x_val, 70), 120)

                        # Update head_y with bounds checking
                        self.head_y = min(max(self.head_y + y_val, 5), 15)

                        print(f"x: {self.head_x}, y: {self.head_y}")

                        self.follow(self.head_x, self.head_y)
                        self.last_time = self.current_time

                        break  # Exit the loop after processing ID 1

    def head_pos(self, posnum):
        headpos = Int16(data=posnum)
        self.pospub.publish(headpos)

    def follow(self, x, y):
        x_msg = Int16(data=x)
        y_msg = Int16(data=y)
        self.posxpub.publish(x_msg)
        self.posypub.publish(y_msg)



if __name__ == '__main__':
    try:
        dps = DetectionPubSub()
        rospy.sleep(0.5)
        dps.head_pos(2)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo("Shutting down gracefully")


