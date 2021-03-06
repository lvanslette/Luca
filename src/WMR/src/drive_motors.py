#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
#import numpy as np
# motor libraries
import time
import RPi.GPIO as GPIO
	
class Motor:
    def __init__(self, fwd, rev, pwm):
        self.fwd = fwd
        self.rev = rev
        self.pwm = pwm

# setup pins for use
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
pins = [19, 26, 23, 24, 22, 27, 21, 20]
PWM_pins = [13, 25, 17, 16]
p = []
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)
for pin in PWM_pins:
    GPIO.setup(pin, GPIO.OUT)
    p.append(GPIO.PWM(pin, 1000))

for i in p:
    i.start(0)

# A: back left, B: back right, C: front left, D: front right
"""
A = Motor(pins[1], pins[2], PWM_pins[1])
motors.append(A)
B = Motor(pins[3], pins[4], PWM_pins[2])
motors.append(B)
C = Motor(pins[5], pins[6], PWM_pins[3])
motors.append(C)
D = Motor(pins[7], pins[8], PWM_pins[4])
motors.append(D)
"""
class Motion_Interface:
    def __init__(self, u):
        self.u = u
        self.PWMs = []
        self.max_duty_cycle = 50
        self.min_duty_cycle = 0
        self.get_PWMs()
        self.set_motors()

    def get_PWMs(self):
        for i in range(4):
            PWM = abs(int(self.u[i] *300));
            if PWM > 50:
                PWM = 50
            # for each wheel, store a PWM value based on angular velocity (m/s)
            self.PWMs.append(PWM)
            sign = 0;
            if self.u[i] != 0:
                sign = self.u[i] / abs(self.u[i])
            print(PWM*sign)

    
    def set_motors(self):
        # change the polarity of the motors depending on forward or backward angular velocity
        for i in range(0, len(pins)-1, 2):
            if self.u[i/2] > 0:
                GPIO.output(pins[i], True)
                GPIO.output(pins[i+1], False)
            else:
                GPIO.output(pins[i], False)
                GPIO.output(pins[i+1], True)

        for i in range(4):
            p[i].ChangeDutyCycle(self.PWMs[i])

            

        
def callback(data):
    # this is where we would take in the array [linear velocity, angular velocity]
    # and output PWMs to each of the motors
    # INPUT: u1,u2,u3,u4, the angular velocities of each motor
    # OUTPUT: convert each to motor PWM values
    u = data.data
    #print(u)
    # convert linear, angular velocity into duty cycle (25-50)
    motion = Motion_Interface(u) 
    """
    print(lin_velocity)
    print(ang_velocity)
    # if we want to turn
    if lin_velocity == 0.0:
    	for i in [0, 4]:
	    GPIO.output(pins[i], True)
	    GPIO.output(pins[i+1], False)
	for i in [2, 6]:
	    GPIO.output(pins[i], False)
	    GPIO.output(pins[i+1], True)
        print("turn")
    else:
	print("forward")
        for i in range(0, len(pins)-1, 2):
	    GPIO.output(pins[i], True)
	    GPIO.output(pins[i+1], False)
        """
def listener():
    rospy.init_node('drive_motors', anonymous=True)
    rospy.Subscriber("echo", Float32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    # if we have quit, then shut down all the motors
    for pin in pins:
        GPIO.output(pin, False)
    GPIO.cleanup()
    quit()
