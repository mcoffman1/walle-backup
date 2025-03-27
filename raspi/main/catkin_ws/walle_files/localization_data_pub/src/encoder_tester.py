import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import numpy as np
from tf.transformations import quaternion_from_euler

class RobotPosePublisher:
    def __init__(self):
        rospy.init_node('robot_pose_publisher')

        # Initialize robot pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.ticks_per_meter = 682
        self.wheel_base = 0.622
        self.wheel_radius = 0.555

        # Subscribe to left_ticks and right_ticks topics
        rospy.Subscriber('left_ticks', Int16, self.left_ticks_callback)
        rospy.Subscriber('right_ticks', Int16, self.right_ticks_callback)

        # Publish current_pose topic
        self.current_pose_pub = rospy.Publisher('current_pose', PoseStamped, queue_size=10)

    def left_ticks_callback(self, msg):
        # Update robot pose based on left wheel ticks
        ticks = msg.data
        wheel_radius = self.wheel_radius
        wheel_distance = self.wheel_base
        distance = float(ticks) / self.ticks_per_meter
        angle = distance / wheel_distance
        self.x += distance * np.cos(self.theta + angle/2)
        self.y += distance * np.sin(self.theta + angle/2)
        self.theta += angle

        # Publish current pose
        self.publish_current_pose()
        rospy.loginfo("Left distance: %.3f", distance)
        rospy.loginfo("Left Ticks: %d", ticks)

    def right_ticks_callback(self, msg):
        # Update robot pose based on right wheel ticks
        ticks = msg.data
        wheel_radius = self.wheel_radius
        wheel_distance = self.wheel_base
        distance = float(ticks) / self.ticks_per_meter
        angle = distance / wheel_distance
        self.x += distance * np.cos(self.theta - angle/2)
        self.y += distance * np.sin(self.theta - angle/2)
        self.theta -= angle

        # Publish current pose
        self.publish_current_pose()
        rospy.loginfo("Right distance: %.3f", distance)
        rospy.loginfo("Right ticks: %d", msg.data)

    def publish_current_pose(self):
        # Publish current pose as PoseStamped message
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(self.x, self.y, 0.0)
        q = quaternion_from_euler(0, 0, self.theta)
        pose.pose.orientation = Quaternion(*q)
        self.current_pose_pub.publish(pose)
        #rospy.loginfo("X position: %d", pose.pose.position.x)

if __name__ == '__main__':
    try:
        rpp = RobotPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
