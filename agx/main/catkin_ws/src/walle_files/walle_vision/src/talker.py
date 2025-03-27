#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
import subprocess
import time
import os
import signal

from arm_posses import ArmPosser


class Talker:
    def __init__(self):
        rospy.init_node('talker')
        launchsub = rospy.Subscriber('launch', Int16, self.launch, queue_size=30, tcp_nodelay=True)
        self.rate = rospy.Rate(3)
        self.process = None
        
        self.ap = ArmPosser()
        
    def launch(self, msg):
        num = msg.data
        
        if num == 1:
            rospy.loginfo("Wall-E")
            command = ["play", "/home/walle/sounds/walle/wall-e.mp3", "vol", "3"]
            
        elif num == 3:
            rospy.loginfo("Eve")
            command = ["play", "/home/walle/sounds/walle/eve.mp3", "vol", "3"]
            subprocess.Popen(command)
            
        elif num == 5:
            rospy.loginfo("Wall-E")
            command = ["play", "/home/walle/sounds/walle/wall-e.mp3", "vol", "5"]
            subprocess.Popen(command)
            self.ap.arm_pos_1()
            
        elif num == 7:
            self.ap.wave()
            
        elif msg.data == 9:
            rospy.loginfo("Power up")
            command = ["play", "/home/walle/sounds/walle/power_up.wav", "vol", "3"]
            subprocess.Popen(command)
            self.ap.arm_pos_0()
            
        elif num == 11:
            rospy.loginfo("Wall-E")
            command = ["play", "/home/walle/sounds/walle/beeping.wav", "vol", "5"]
            subprocess.Popen(command)
            
        elif num == 12:
            rospy.loginfo("Wall-E")
            command = ["play", "/home/walle/sounds/walle/beeping.wav", "vol", "3"]
            subprocess.Popen(command)
            

if __name__ == '__main__':
    try:
        talk = Talker()
        while not rospy.is_shutdown():
            talk.rate.sleep()
    except rospy.ROSInterruptException:
        pass
