#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
import subprocess
import time

class Launcher:
    def __init__(self):
        rospy.init_node('piLauncher')
    
        launchsub = rospy.Subscriber('launch', Int16, self.launch, queue_size=10)
        
        self.rate = rospy.Rate(10)
        
    def launch(self, msg):
        num = msg.data
        if num == 0: self.shutdown()
        
    def shutdown(self):
        time.sleep(10)  # Wait for 10 seconds
        command = "shutdown -h now"
        subprocess.run(command, shell=True)
        
if __name__=='__main__':
    try:
        laun = Launcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        

