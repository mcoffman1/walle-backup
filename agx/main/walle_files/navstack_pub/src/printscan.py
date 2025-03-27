import rospy
from sensor_msgs.msg import LaserScan

class ScanFilter:
    def __init__(self):
        rospy.init_node('scan_print', anonymous=True)
        rospy.Subscriber("/walle_scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        self.print_header(msg)
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + (i * msg.angle_increment)
            print("({},{},{})".format(i,angle, r))
            
    def print_header(self, msg):
        for attr in msg.__slots__:
            if attr == 'ranges' or attr == 'intensities':
                continue
            value = getattr(msg, attr)
            print(attr, value)

if __name__ == '__main__':
    sf = ScanFilter()
    rospy.spin() # Keeps code from exiting until the node is stopped

