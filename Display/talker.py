#!/usr/bin/env python3
# *** talker.py ***
# sends a message at the specified rate to listener.py
import rospy
import time
from std_msgs.msg import Float32MultiArray

def talker():
    pub = rospy.Publisher('chatter', Float32MultiArray, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    count = 0
    data = Float32MultiArray()
    while not rospy.is_shutdown():
        data.data = [.1, .1, count%5, count%6]
        #coordinates = str(count%5)+str(count%6)
        count +=1
        pub.publish(data)
        print(data.data)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
