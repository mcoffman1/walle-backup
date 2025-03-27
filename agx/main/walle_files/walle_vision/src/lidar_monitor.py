import rospy
import subprocess
from std_msgs.msg import Int16


class LidarMonitor:
    def __init__(self):
        rospy.init_node('lidar_monitor', anonymous=True)
        
        self.lidar_process = None
        self.desired_state = "WAIT"
        self.MAX_CONSECUTIVE_FAILURES = 3
        self.consecutive_failures = 0
        
        rospy.Subscriber("/launch", Int16, self.launch_callback)
    
    def start_lidar_node(self):
        cmd = ["roslaunch", "rplidar_ros", "rplidar_a1.launch"]
        self.lidar_process = subprocess.Popen(cmd)
        rospy.loginfo("LIDAR started")

    def stop_lidar_node(self):
        if self.lidar_process:
            self.lidar_process.terminate()
            self.lidar_process = None
        rospy.loginfo("LIDAR stopped")
    
    def launch_callback(self, msg):
        if msg.data == 9 and self.desired_state != "RUN":
            self.desired_state = "RUN"
            self.start_lidar_node()
        elif msg.data == 10 and self.desired_state != "WAIT":
            self.desired_state = "STOP"
            self.stop_lidar_node()
    
    def monitor_lidar(self):
        while not rospy.is_shutdown() and self.desired_state == "RUN":
            # Logic to monitor the LIDAR and restart if necessary
            # Placeholder - replace with actual logic
            rospy.sleep(5)
            
    def run(self):
        while not rospy.is_shutdown():
            if self.desired_state == "RUN":
                self.monitor_lidar()
            else:
                rospy.sleep(5)


if __name__ == "__main__":
    monitor = LidarMonitor()
    monitor.run()

