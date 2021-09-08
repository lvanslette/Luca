# A program to control the movement of a single motor using the RTK MCB!
# Composed by The Raspberry Pi Guy to accompany his tutorial!

# Let's import the modules we will need!
import time
import RPi.GPIO as GPIO

# Next we setup the pins for use!
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
# reate array of numbers corresponding to the pins on the raspberry pi
# pins = [back left forward, back left reverse, back right forward, back right reverse, 
#           front right forward, front right reverse, front left forward, front left reverse]
pins = [19, 26, 23, 24, 22, 27, 21, 20]
for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
for pin in pins:
    GPIO.output(pin, False)
GPIO.cleanup()
