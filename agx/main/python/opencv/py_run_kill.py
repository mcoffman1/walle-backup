import rospy
from std_msgs.msg import Int16
import subprocess


class Initializer:
    def __init__(self):
        rospy.init_node('launcher')
        launchsub = rospy.Subscriber("launcher", Int16, self.launch, queue_size=100, tcp_nodelay=True)
        self.rate = rospy.Rate(30)
        self.process = None

    def launch(self, msg):
        num = msg.data

        if num == 1:
            self.runtest()
        elif num == 2:
            self.killtest()

    def runtest(self):
        # Specify the path to your Bash script
        script_path = '/home/student/catkin_ws/src/walle_vision/src/test.py'

        # Execute the Bash script and store the process object
        self.process = subprocess.Popen(['python', script_path])

    def killtest(self):
        # Retrieve the PID of the rospy node
        output = subprocess.check_output(['pgrep', '-f', 'test.py'])
        pids = output.decode().split()  # Convert bytes to string and remove trailing newline
        pid = pids[0] if pids else None

        # Terminate the rospy node
        subprocess.run(['kill', pid])

        print(pid)


    def cleanup(self):
        # Cleanup resources if needed
        pass

if __name__ == '__main__':
    try:
        init = Initializer()
        while not rospy.is_shutdown():
            init.rate.sleep()
    except rospy.ROSInterruptException:
        pass

