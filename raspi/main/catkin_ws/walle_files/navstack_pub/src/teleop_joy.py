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
        self.angular_scale = rospy.get_param("~angular_scale", 3.0)
        
        # Button press flags
        self.A_held = False
        self.B_held = False
        self.X_held = False
        self.Y_held = False
        self.sel_held = False
        self.start_held = False
        self.LB_held = False
        self.RB_held = False

        self.joystick = True



    def joy_callback(self, joy_msg):
        # Create a Twist message
        twist = Twist()

        # Set linear and angular velocities based on joystick input
        twist.linear.x = self.linear_scale * joy_msg.axes[self.linear_axis]
        twist.angular.z = self.angular_scale * joy_msg.axes[self.angular_axis]

        # Publish the Twist message if joystick is active
        if self.joystick:
            self.cmd_vel_pub.publish(twist)

        # Check if joystick is in a neutral position (close to 0 on both axes)
        if -0.1 < joy_msg.axes[self.linear_axis] < 0.1 and -0.1 < joy_msg.axes[self.angular_axis] < 0.1:
            if self.joystick:
                self.stop()  # Call the stop function
            self.joystick = False  # Set joystick to inactive when in neutral
        else:
            self.joystick = True  # Activate joystick when there is movement

        # Check buttons
        self.button_check(joy_msg)
        
        

    def button_check(self, joy_msg):
    
        # Check if button 0 (A) is pressed
        if joy_msg.buttons[0] == 1:
            if not self.A_held:
                self.A_held = True
                self.launch_pub.publish(3)  
        else:
            self.A_held = False

        # Check if button 1 (B) is pressed
        if joy_msg.buttons[1] == 1:
            if not self.B_held:
                self.B_held = True
                self.launch_pub.publish(4)  
        else:
            self.B_held = False

        # Check if button 3 (X) is pressed
        if joy_msg.buttons[3] == 1:
            if not self.X_held:
                self.X_held = True
                self.launch_pub.publish(1)
        else:
            self.X_held = False

        # Check if button 4 (Y) is pressed
        if joy_msg.buttons[4] == 1:
            if not self.Y_held:
                self.Y_held = True
                self.launch_pub.publish(2)
        else:
            self.Y_held = False
            
        # Check if button 6 (LB) is pressed
        if joy_msg.buttons[6] == 1:
            if not self.LB_held:
                self.LB_held = True
                self.launch_pub.publish(11)
        else:
            self.LB_held = False
            
        # Check if button 7 (RB) is pressed
        if joy_msg.buttons[7] == 1:
            if not self.RB_held:
                self.RB_held = True
                self.launch_pub.publish(12)
        else:
            self.RB_held = False
            
        # Check if button 10 (select) is pressed
        if joy_msg.buttons[10] == 1:
            if not self.sel_held:
                self.sel_held = True
                self.launch_pub.publish(5)
        else:
            self.sel_held = False
            
        # Check if button 11 (start) is pressed
        if joy_msg.buttons[11] == 1:
            if not self.start_held:
                self.start_held = True
                self.launch_pub.publish(9)
        else:
            self.start_held = False
            
            
    def stop(self):
        # Create a Twist message with all zeros
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.linear.y = 0.0
        stop_twist.linear.z = 0.0
        stop_twist.angular.x = 0.0
        stop_twist.angular.y = 0.0
        stop_twist.angular.z = 0.0

        # Publish the stop message
        self.cmd_vel_pub.publish(stop_twist)
        
        

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        teleop = JoyTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass

