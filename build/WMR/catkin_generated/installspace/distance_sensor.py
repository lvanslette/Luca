#!/usr/bin/env python2

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Float32

def get_distance():
    try:
        # using the GPIO numbers, not the pins themselves
        GPIO.setmode(GPIO.BCM)

        # this pin starts the sensor
        PIN_TRIGGER = 18
        # echo pin is where we get out data from
        PIN_ECHO = 5

        # expect an input from echo, output from trigger
        GPIO.setup(PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(PIN_ECHO, GPIO.IN)

        # set the pin to LOW to let the sensor settle
        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        print("waiting for sensor to settle...")

        time.sleep(0.5)

        print("calculating distance:")
        # set TRIGGER pin to HIGH to start the sensor
        GPIO.output(PIN_TRIGGER, GPIO.HIGH)

        time.sleep(0.00001)
        # immediately set it to low after 1 ns
        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        # while the ECHO pin is LOW, we are setting the pulse start time until it becomes HIGH
        while GPIO.input(PIN_ECHO) == 0:
            pulse_start_time = time.time()

        # once the echo pin reads HIGH, we then set the end time to the current time until ECHO is set back to LOW
        while GPIO.input(PIN_ECHO) == 1:
            pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        distance = round(pulse_duration * 17150, 2)
        print("Distance:", distance, "cm")
        return distance

    finally:
        GPIO.cleanup()

def talker():
    pub = rospy.Publisher('measurement', Float32, queue_size=1000)
    rospy.init_node('distance_sensor', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        distance = get_distance()
        rospy.loginfo(distance)
        pub.publish(distance)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

