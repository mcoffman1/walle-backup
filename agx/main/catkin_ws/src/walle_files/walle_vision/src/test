#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
import subprocess
import time
import os
import signal


class Launcher:
    def __init__(self):
        rospy.init_node('launcher')
        
        self.pospub = rospy.Publisher('head_pos', Int16, queue_size=10)
        self.doorpub = rospy.Publisher('door_pos', Int16, queue_size=10)
        launchsub = rospy.Subscriber('launch', Int16, self.launch, queue_size=100, tcp_nodelay=True)
        
        self.rate = rospy.Rate(10)
        self.process = None
        self.processrunning = 0
        self.rplidar_process = None

    def launch(self, msg):
        num = msg.data
        
        if num == 0:
            if self.processrunning == 0:
                self.head_pose0()
            elif self.processrunning == 1:
                self.killtest(1)
                rospy.sleep(3)
                self.head_pose0()

        elif num == 1:
            rospy.loginfo("looking for people")
            self.runtest(1)
            
        elif num == 2:
            rospy.loginfo("shutting down face tracker")
            self.killtest(1)
            
        ## If msg is 9 and RPLIDAR isn't running, launch it
        #elif msg.data == 9 and self.rplidar_process is None:
            #launch_command = "roslaunch rplidar_ros rplidar_a1.launch"
            #self.rplidar_process = subprocess.Popen(launch_command, shell=True, preexec_fn=os.setsid)
            #rospy.loginfo("Attempted to launch the RPLIDAR node")

        ## If msg is 10 and the RPLIDAR node is running, kill it
        #elif msg.data == 10 and self.rplidar_process is not None:
            #rospy.loginfo("Attempted to kill the RPLIDAR node")
            #rospy.sleep(2)
            ## ensure all child processes are also terminated
            #os.killpg(os.getpgid(self.rplidar_process.pid), signal.SIGTERM)
            #self.rplidar_process = None
            
        elif num == 11:
            rospy.loginfo("open door")
            door_msg = Int16(data=100)
            self.doorpub.publish(door_msg)
            
        elif num == 12:
            rospy.loginfo("close door")
            door_msg = Int16(data=10)
            self.doorpub.publish(door_msg)
            
            
            
    def runtest(self, script):
        try:   
            if script == 1:
                # Specify the path to your Bash script
                script_path = '/home/walle/catkin_ws/src/walle_files/walle_vision/src/detectnet_coord_pub.py'
                self.processrunning = 1

            # Execute the Bash script and store the process object
            self.process = subprocess.Popen(['python', script_path])
            
        except FileNotFoundError:
            print(f"Script not found at {script_path}")
        except Exception as e:
            print(f"ERROR: {e}")
            
            

    def killtest(self, script):
        if script == 1:
            # Retrieve the PID of the rospy node
            output = subprocess.check_output(['pgrep', '-f', 'detectnet_coord_pub.py'])
            
        pids = output.decode().split()  # Convert bytes to string and remove trailing newline
        pid = pids[0] if pids else None

        # Terminate the rospy node
        subprocess.run(['kill', pid])
        
        self.head_pose1()
        
        print(pid)
        
        
    
    def head_pose1(self):
        pos_msg = Int16(data=2)
        self.pospub.publish(pos_msg)
        time.sleep(1)
        pos_msg = Int16(data=1)
        self.pospub.publish(pos_msg)
        time.sleep(0.5)
        self.processrunning = 0
           
    def head_pose0(self):
        pos_msg = Int16(data=0)
        self.pospub.publish(pos_msg)
        self.shutdown()
        
    def shutdown(self):
        time.sleep(10)
        command = 'shutdown -h now'
        subprocess.run(command, shell=True)

    def cleanup(self):
        # Cleanup resources if needed
        pass

if __name__ == '__main__':
    try:
        laun = Launcher()
        while not rospy.is_shutdown():
            laun.rate.sleep()
    except rospy.ROSInterruptException:
        pass

