#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
import time

## Set up publishers to send commands to arm controls
    # left_arm_pos
    # left_shoulder_out_pos
    # left_shoulder_pos
    # left_wrist_pos    
    
    # right_arm_pos
    # right_shoulder_out_pos
    # right_shoulder_pos
    # right_wrist_pos
    # right_hand_pos

class ArmPosser:
    def __init__(self):
        rospy.init_node('arm_posser')

        self.la_pub = rospy.Publisher('left_arm_pos', Int16, queue_size = 10)
        self.lso_pub = rospy.Publisher('left_shoulder_out_pos', Int16, queue_size = 10)
        self.ls_pub = rospy.Publisher('left_shoulder_pos', Int16, queue_size = 10)
        self.lw_pub = rospy.Publisher('left_wrist_pos', Int16, queue_size = 10)

        self.ra_pub = rospy.Publisher('right_arm_pos', Int16, queue_size = 10)
        self.rso_pub = rospy.Publisher('right_shoulder_out_pos', Int16, queue_size = 10)
        self.rs_pub = rospy.Publisher('right_shoulder_pos', Int16, queue_size = 10)
        self.rw_pub = rospy.Publisher('right_wrist_pos', Int16, queue_size = 10)
        self.rh_pub = rospy.Publisher('right_hand_pos', Int16, queue_size = 10)

        self.rate = rospy.Rate(.5)

    def arm_pos_0(self):
        sho_out = Int16(data=20)
        arm_ac = Int16(data=0)
        sho = Int16(data=35)
        wrist = Int16(data=0)
        hand = Int16(data=0)

        rospy.sleep(.2)
        self.lso_pub.publish(sho_out)
        self.rso_pub.publish(sho_out)
        rospy.sleep(.5)
        self.la_pub.publish(arm_ac)
        self.ra_pub.publish(arm_ac)
        self.ls_pub.publish(sho)
        self.rs_pub.publish(sho)
        self.lw_pub.publish(wrist)
        self.rw_pub.publish(wrist)
        self.rh_pub.publish(hand)

    def arm_pos_1(self):
        sho_out = Int16(data=20)

        left_sho_out = Int16(data=10)
        right_sho_out = Int16(data=5) 

        arm_ac = Int16(data=20)
        arm_ac2 = Int16(data=0)

        sho = Int16(data=70)
        wrist = Int16(data=100)
        hand = Int16(data=0)

        rospy.sleep(.2)
        self.lso_pub.publish(sho_out)
        self.rso_pub.publish(sho_out)
        rospy.sleep(.5)
        self.la_pub.publish(arm_ac)
        self.ra_pub.publish(arm_ac)
        rospy.sleep(.5)
        self.ls_pub.publish(sho)
        self.rs_pub.publish(sho)
        self.lw_pub.publish(wrist)
        self.rw_pub.publish(wrist)
        self.rh_pub.publish(hand)
        rospy.sleep(1)
        self.lso_pub.publish(left_sho_out)
        self.rso_pub.publish(right_sho_out)
        rospy.sleep(.5)
        self.la_pub.publish(arm_ac2)
        self.ra_pub.publish(arm_ac2)

    def arm_pos_2(self):
        sho_out = Int16(data=20)

        left_sho_out = Int16(data=10)
        right_sho_out = Int16(data=5) 

        arm_ac = Int16(data=20)
        arm_ac2 = Int16(data=0)

        sho = Int16(data=60)
        wrist = Int16(data=0)
        hand = Int16(data=0)

        rospy.sleep(.2)
        self.lso_pub.publish(sho_out)
        self.rso_pub.publish(sho_out)
        rospy.sleep(.5)
        self.la_pub.publish(arm_ac)
        self.ra_pub.publish(arm_ac)
        rospy.sleep(.5)
        self.ls_pub.publish(sho)
        self.rs_pub.publish(sho)
        self.lw_pub.publish(wrist)
        self.rw_pub.publish(wrist)
        self.rh_pub.publish(hand)
        rospy.sleep(1)
        self.lso_pub.publish(left_sho_out)
        self.rso_pub.publish(right_sho_out)
        rospy.sleep(.5)
        self.la_pub.publish(arm_ac2)
        self.ra_pub.publish(arm_ac2)

    def wave(self):
        sho_out1 = Int16(data=20)
        sho_out2 = Int16(data=5)
        arm_ac1 = Int16(data=50)
        sho1 = Int16(data=120)
        wrist1 = Int16(data=90)
        hand1 = Int16(data=100)
        hand2 = Int16(data=0)

        rospy.sleep(.2)
        self.rso_pub.publish(sho_out1)
        rospy.sleep(.5)
        self.ra_pub.publish(arm_ac1)
        rospy.sleep(.5)
        self.rs_pub.publish(sho1)
        rospy.sleep(.5)
        self.rso_pub.publish(sho_out2)
        self.rw_pub.publish(wrist1)
        self.rh_pub.publish(hand1)
        rospy.sleep(1)
        self.rh_pub.publish(hand2)
        rospy.sleep(.5)
        self.rh_pub.publish(hand1)
        rospy.sleep(.5)
        self.rh_pub.publish(hand2)
        rospy.sleep(.5)
        self.rh_pub.publish(hand1)
        rospy.sleep(.5)


if __name__=='__main__':
    try:
        app = ArmPosser()
        first = True
        while not rospy.is_shutdown():
            if first:
                app.arm_pos_0()
                first=False
            app.rate.sleep()
    except rospy.ROSIneruptException:
        pass



