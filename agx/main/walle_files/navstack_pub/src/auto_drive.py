#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from filter import ScanFilter
import math

class ObstacleAvoidance:
    def __init__(self):
        sf = ScanFilter()
        self.footprint = 0.5
        self.max_turn = 2.0
        self.min_dist = 0.3
        self.stop = True
        scan_sub = rospy.Subscriber('/walle_scan', LaserScan, self.scan_callback)
        launch_sub = rospy.Subscriber('/launch', Int16, self.launch_callback, queue_size=10, tcp_nodelay=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10, latch=True)
        self.twist = Twist()
       
        self.mode = 'forward'
        self.last_dir = 'forward'
        self.backup = False
        self.turn_count = 0
        
        self.maxspeed = 0.75

    def launch_callback(self, launch_data):
        num = launch_data.data
        if num == 3:
            self.stop = False
        elif num == 4:
            self.stop_robot()
            self.stop = True


    def scan_callback(self, scan_data):
        closest_range = min(scan_data.ranges)
        if self.stop:
            rospy.sleep(0.2)
        elif closest_range < 0.31:
            if self.mode == 'back':
                self.twist.angular.z = 0.0
                self.twist.linear.x = -0.5
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.mode = 'back'
        else:
            # determine closest point in scan data
            left_range_values = []
            right_range_values = []

            for i,r in enumerate(scan_data.ranges):
                current_angle = scan_data.angle_min + i * scan_data.angle_increment
                if r > self.footprint/math.cos(abs(current_angle)-(math.pi/2)): r = 47
                left_range_values.append(r) if current_angle > 0 else right_range_values.append(r)
                #print(f'Left angle: {current_angle}') if ind_dir == 'left' else print(f'Right angle: {current_angle}')

            closest_left = min(left_range_values)
            left_index = scan_data.ranges.index(closest_left)
            left_angle = scan_data.angle_min + left_index * scan_data.angle_increment
            
            closest_right = min(right_range_values)
            right_index = scan_data.ranges.index(closest_right)
            right_angle = scan_data.angle_min + right_index * scan_data.angle_increment
            
            #print(f'left angle: {left_angle}  |  right angle{right_angle}')
            #print(f'left range: {closest_left}  |  right range: {closest_right}')
            self.set_mode(left_angle, right_angle, closest_left, closest_right)

            
            
    def set_mode(self, left_angle, right_angle, left_range, right_range):
        
            #print(f'left: {left_angle}  |  right: {right_angle}')
            angle = left_angle if left_range < right_range else -right_angle
            closest_range = left_range if left_range < right_range else right_range
            direction = 'right' if left_range < right_range else 'left'

            # if outside footprint or great than 2 meters(max_turn) move forward
            if closest_range > self.max_turn:
                if self.backup and self.turn_count < 5:
                    self.mode = 'right' if self.last_dir == 'right' else 'left'
                    self.turn_count += 1
                else:
                    self.mode = 'forward'
                    self.backup = False
                    self.turn_count = 0

            # if les than .3 meters(min_dist) back up
            elif closest_range < self.min_dist/math.cos(math.pi-angle):
                if self.mode == 'back':
                    self.backup = True
                    self.last_dir = direction
                self.mode = 'back'

            # if object in range and no to close turn
            # if already turning continue same direction to avoid back and forth
            else:
                if self.mode == 'right' or self.mode == 'left':
                    pass
                else:
                    self.mode = direction
            self.walle_go(angle,closest_range)
        
        
    def walle_go(self,scan_angle,scan_range):
        # determine turn_direction from closest obstacle
        if self.mode == 'forward':
            self.twist.angular.z = 0.0
            self.twist.linear.x = self.maxspeed
            print('-----------------Forward--------------------')
            print(f'angular: {self.twist.angular.z}  |  linear:  {self.twist.linear.x}')
        elif self.mode == 'back':
            self.twist.linear.x = -0.5
            if self.backup:
                self.twist.angular.z = 0.0
                print('----------------- Back ---------------------')
                print(f'angular: {self.twist.angular.z}  |  linear:  {self.twist.linear.x}')
        elif self.mode == 'right':
            self.twist.angular.z = (-(scan_angle - 1.5)*2)/(scan_range + 0.3)
            self.twist.linear.x = min((scan_range)-0.4, self.maxspeed)
            print(f'---------Right count: {self.turn_count}----------')
            print(f'angular: {self.twist.angular.z}  |  linear:  {self.twist.linear.x}')
        else:# self.mode == 'left':
            self.twist.angular.z = ((scan_angle - 1.5)*2)/(scan_range + 0.3)
            self.twist.linear.x = min((scan_range)-0.4, self.maxspeed)
            print(f'------Left count: {self.turn_count}-------------')
            print(f'angular: {self.twist.angular.z}  |  linear:  {self.twist.linear.x}')

        self.cmd_vel_pub.publish(self.twist)
    
        

    def stop_robot(self):
        self.twist.angular.z = 0.0
        self.twist.linear.x = 0.0
        # publish robot velocity
        self.cmd_vel_pub.publish(self.twist)

            
               
if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance')
    oav = ObstacleAvoidance()
    rospy.on_shutdown(oav.stop_robot)
    #oav.start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rospy.loginfo("Robot stopped.")


