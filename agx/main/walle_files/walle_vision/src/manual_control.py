import rospy
from std_msgs.msg import Int16


rospy.init_node('manual_control', anonymous=True)

posxpub = rospy.Publisher('head_x', Int16, queue_size=10)
posypub = rospy.Publisher('head_y', Int16, queue_size=10)
pospub = rospy.Publisher('head_pos', Int16, queue_size=10)

speedxpub = rospy.Publisher('speed_x', Int16, queue_size=10)
speedypub = rospy.Publisher('speed_y', Int16, queue_size=10)

pospos = 1
first = True


def main():
    global first
    print('Enter head position')
    pospos = int(input('>>'))
    
    if pospos < 3:
        pospos_msg = Int16(data=pospos)
        pospub.publish(pospos_msg)
    
        if pospos == 2:
            if first:
                #first = False
                speedx = int(input('speed -->>'))
                posx = int(input('x -->>'))
                posy = int(input('y -->>'))

                speedx_msg = Int16(data=speedx)
                posx_msg = Int16(data=posx)
                posy_msg = Int16(data=posy)
                
                speedxpub.publish(speedx_msg)
                posxpub.publish(posx_msg)
                posypub.publish(posy_msg)
            else:
                posx = int(input('x -->>'))

                posx_msg = Int16(data=posx)
                
                posxpub.publish(posx_msg)
                
    return pospos
                
            
if __name__=='__main__':
    while True:
        var = main()
        if var >= 3:
            print('Exiting program')
            break
