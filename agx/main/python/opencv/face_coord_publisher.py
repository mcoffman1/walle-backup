#!/usr/bin/env python

import rospy
import cv2 as cv
from std_msgs.msg import Int16, Empty
from face_tracker import FaceTracker
from coord_calc import SpeedCalculator
import argparse
import cv2
import time


class FaceCoordPub:
    def __init__(self):
        rospy.init_node('face_coord_pub', anonymous=True)

        self.ft = FaceTracker()
        self.cs = SpeedCalculator()
        self.cs.plotter()
                
        self.posxpub = rospy.Publisher('head_x', Int16, queue_size=10)
        self.posypub = rospy.Publisher('head_y', Int16, queue_size=10)
        self.speedxpub = rospy.Publisher('speed_x', Int16, queue_size=10)
        self.speedypub = rospy.Publisher('speed_y', Int16, queue_size=10)

        self.pospub = rospy.Publisher('head_pos', Int16, queue_size=10)

        self.rate = rospy.Rate(60)

        time.sleep(2)  # wait for the serial connection to establish

        self.nofacecount = 0
        #self.xpwm = 0
        self.ypwm = 40
        
        
    def get_arguments(self):
        ap = argparse.ArgumentParser()
        ap.add_argument('-d', '--display', required=False,
                        help='Display the video feed on the desktop', action='store_true')
        args = vars(ap.parse_args())
        return args


    def map(self, value, start1, stop1, start2, stop2):
        return start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1))
        
    def main(self):
        #self.nofacecount
        self.nofacecount += 1
        x, y = self.ft.track_face()
        if x < 0 or y < 0:
            if self.nofacecount > 10:
                self.nofacecount = 0
                self.send(100, 100, 95, 15)

            else:
                self.nofacecount += 1
                self.send(self.cs.xspeed, self.cs.yspeed, 0, 0)
        else:
            self.cs.calc_speed(x, y)
            xpos = self.map(x, 0, 640, 4, -4)
            if xpos > 1: xpos = 1
            elif xpos < -1: xpos = -1
            else: xpos = 0
            ypos = self.map(y, 0, 340, -4, 4)
            if ypos > 1: ypos = 1
            elif ypos < -1: ypos = -1
            else: ypos = 0
            self.send(self.cs.xspeed, self.cs.yspeed, xpos, ypos)
            self.nofacecount = 0


    def send(self, speedx, speedy, posx, posy):
        speedx = int(speedx)
        speedy = int(speedy)
        posx = int(posx)
        posy = int(posy)

        speedx_msg = Int16(data=speedx)
        speedy_msg = Int16(data=speedy)
        self.speedxpub.publish(speedx_msg)
        self.speedypub.publish(speedy_msg)
        
        posx_msg = Int16(data=posx)
        posy_msg = Int16(data=posy)
        self.posxpub.publish(posx_msg)
        self.posypub.publish(posy_msg)

        
if __name__ == '__main__':
    try:
        fcp = FaceCoordPub()
        args = fcp.get_arguments()
        if args['display']:
            fcp.ft.display = True
        pos_msg = Int16(data=2)
        fcp.pospub.publish(pos_msg)
        while not rospy.is_shutdown():
            fcp.main()
            fcp.rate.sleep()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # Release the resources
        fcp.ft.cap.stop()
        cv2.destroyAllWindows()
        pos_msg = Int16(data=2)
        fcp.pospub.publish(pos_msg)
        time.sleep(2)
        pos_msg = Int16(data=1)
        fcp.pospub.publish(pos_msg)


        
        
    except rospy.ROSInterruptException:
        pass
