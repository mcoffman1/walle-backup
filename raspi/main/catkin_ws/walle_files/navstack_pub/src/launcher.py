#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class JoyTeleop:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('joy_teleop', anonymous=True)

        # Define the publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.launch_pub = rospy.Publisher('/launch', Int16, queue_size=10)
        
        # Define the subscriber for joy
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        # Parameters for joystick control
        self.linear_axis = rospy.get_param("~linear_axis", 1)  # Default to axis 1 (left stick Y for many controllers)
        self.angular_axis = rospy.get_param("~angular_axis", 0)  # Default to axis 0 (left stick X for many controllers)
        self.linear_scale = rospy.get_param("~linear_scale", 1.4)  # Set max speed to 1.4
        self.angular_scale = rospy.get_param("~angular_scale", 3.5)
        
        # Button press flags
        self.A_held = False
        self.B_held = False
        self.X_held = False
        self.Y_held = False

    def joy_callback(self, joy_msg):
        # Create a Twist message
        twist = Twist()

        # Set linear and angular velocities based on joystick input
        twist.linear.x = self.linear_scale * joy_msg.axes[self.linear_axis]
        twist.angular.z = self.angular_scale * joy_msg.axes[self.angular_axis]

        # Publish the Twist message
        self.cmd_vel_pub.publish(twist)

        # Check buttons
        self.button_check(joy_msg)

    def button_check(self, joy_msg):
        # Check if button 0 (A) is pressed
        if joy_msg.buttons[0] == 1:
            if not self.A_held:
                self.A_held = True
                print("What the heck")
                self.launch_pub.publish(1)  # Publish integer 1 when A is pressed
        else:
            self.A_held = False

        # Check if button 1 (B) is pressed
        if joy_msg.buttons[1] == 1:
            if not self.B_held:
                self.B_held = True
                self.launch_pub.publish(2)  # Publish integer 2 when B is pressed
        else:
            self.B_held = False

        # Check if button 2 (X) is pressed
        if joy_msg.buttons[2] == 1:
            if not self.X_held:
                self.X_held = True
                self.launch_pub.publish(3)  # Publish integer 3 when X is pressed
        else:
            self.X_held = False

        # Check if button 3 (Y) is pressed
        if joy_msg.buttons[3] == 1:
            if not self.Y_held:
                self.Y_held = True
                self.launch_pub.publish(4)  # Publish integer 4 when Y is pressed
        else:
            self.Y_held = False

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        teleop = JoyTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

