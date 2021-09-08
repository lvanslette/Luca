#!/usr/bin/env python2

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32MultiArray


# using the GPIO numbers, not the pins themselves
GPIO.setmode(GPIO.BCM)
# TODO: add GPIO pins 
FRONT = 1
BACK = 2
LEFT = 3
RIGHT = 4
GPIO.setup(FRONT, GPIO.IN)
GPIO.setup(BACK, GPIO.IN)
GPIO.setup(LEFT, GPIO.IN)
GPIO.setup(RIGHT, GPIO.IN)
# using the IR sensors, determine which sensors trigger something nearby
def get_warnings():
    try:

        # TODO: can the GPIO input return a 1 or 0?
        #   warnings = [ GPIO.input(FRONT), GPIO.input(BACK), GPIO.input(LEFT), GPIO.input(RIGHT) ]
        #   print(warnings)
        time.sleep(0.5)

    finally:
        GPIO.cleanup()

def talker():
    pub = rospy.Publisher('measurement', Int32MultiArray, queue_size=1000)
    rospy.init_node('ir_sensors', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        warnings = get_warnings()
        rospy.loginfo(warnings)
        pub.publish(warnings)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

