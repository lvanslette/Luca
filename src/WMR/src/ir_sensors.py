#!/usr/bin/env python

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32MultiArray


# using the GPIO numbers, not the pins themselves
GPIO.setmode(GPIO.BCM)
# HOW TO COMMUNICATE
# GPS: get GPS coordinates, get other robot's coordinates, adjust location based on other data
FRONT = 5
GPIO.setup(FRONT, GPIO.IN)

BACK = 4
GPIO.setup(BACK, GPIO.IN)

LEFT = 18
RIGHT = 12
GPIO.setup(LEFT, GPIO.IN)
GPIO.setup(RIGHT, GPIO.IN)

# using the IR sensors, determine which sensors trigger something nearby
def get_warnings():
    try:

        # have to do not() because it returns the opposite of what the resulting measurement is
        # warnings = [ GPIO.input(FRONT), GPIO.input(BACK), GPIO.input(LEFT), GPIO.input(RIGHT) ]
        warnings = [not(GPIO.input(FRONT)), not(GPIO.input(BACK)), not(GPIO.input(LEFT)), not(GPIO.input(RIGHT))]
        time.sleep(0.1)
        return warnings
    finally:
        print(warnings)

def talker():
    pub = rospy.Publisher('measurement', Int32MultiArray, queue_size=1000)
    rospy.init_node('ir_sensors', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        warnings = Int32MultiArray(data=get_warnings())
        pub.publish(warnings)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass

