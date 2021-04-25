#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

import wiringpi as wpi

def talker():
    wpi.wiringPiSetup()
    wpi.pinMode(1, 0)
    old_state = 1 
    pub = rospy.Publisher('io1', String, queue_size=10)
    rospy.init_node('rosio', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        data = wpi.digitalRead(1)
        if old_state == data:
            print(data)
        else:
            print(data)
            if data == 0:
                print("down")
                pub.publish("down")
            else:
                print("up")
                pub.publish("up")
            old_state = data

        
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
