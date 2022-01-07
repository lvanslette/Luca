#!/usr/bin/env python3

import time
import math   # math.pi
import threading
import pickle
from threading import Thread,Lock
from functools import partial
import signal

dist = 0
# counters: the total counts since the start of the program
counters = [0, 0, 0, 0]
# prev_counters: the total counts since the last time the 
#                wheel velocity was calculated
prev_counters = [0, 0, 0, 0]
# vels: the current wheel velocities, must factor in a delay to get   
#       a more precise measurement of wheel velocities
vels = [0,0,0,0]

import smbus
DEVICE = 0x20   # used to reference the MCP23017 chip
IODIRA = 0x00   # sets the direction (input/output) of each of the A pins
GPIOA = 0x12    # read this register when you want to read pins from A
OLATA = 0x14    # read this register when you want to write to pins
IODIRB = 0x01    # same for B side
GPIOB = 0x13    # same for B side
OLATB = 0x15    # same for B side

import socket
HOST = socket.gethostbyname('ubuntu_ros_1')  # server hostname or IP address
#HOST = '0.0.0.0'  # server hostname or IP address
PORT = 80        # port used by server
  

def bus_setup():
  bus = smbus.SMBus(1)
  # 00001111 = 0x0F, set GPB0-GPB3 as input
  # IODIRB: encoders
  bus.write_byte_data(DEVICE, IODIRB, 0x0F)
  # 1011 0111 = 0x7D, set GPA0,GPA6 as output, rest input
  # IODIRA: for the ir sensors and distance sensor
  bus.write_byte_data(DEVICE, IODIRA, 0xB7)
  # output all to 0 or no?
  #bus.write_byte_data(DEVICE, OLATA, 0x00)
  return bus

def get_distance(bus):
  # TRIG: GPA6, output
  TRIG_OUTPUT_HIGH = 0x40
  TRIG_OUTPUT_LOW = 0x00
  # ECHO: GPA7, input
  # initially set TRIG to false
  bus.write_byte_data(DEVICE, OLATA, TRIG_OUTPUT_LOW)
  time.sleep(0.5)
  # set TRIG to HIGH (0010 0000)
  bus.write_byte_data(DEVICE, OLATA, TRIG_OUTPUT_HIGH)
  time.sleep(0.00001)
  bus.write_byte_data(DEVICE, OLATA, TRIG_OUTPUT_LOW)

  # since ECHO is on GPA7, which is 128 in decimal, 
  # ECHO=0 will return a number less than 128,
  # and ECHO=1 will return a number >= 128
  pulse_start = 0
  pulse_end = 0
  while bus.read_byte_data(DEVICE,GPIOA) < 128:
    pulse_start = time.time()
  while bus.read_byte_data(DEVICE, GPIOA) >= 128:
    pulse_end = time.time()
  # if we are too close, doesnt have enough time to read the i2c bus 
  # before the pin goes low again
  #  --> after the pin goes HIGH (>=128) exits the first while loop, but before the
  #      pin can be read again the pin goes LOW (<128) so the second while loop is
  #      never executed
  if pulse_end == 0:
    pulse_duration = 0
  else:
    pulse_duration = pulse_end - pulse_start

  distance = pulse_duration * 17150
  distance = round(distance, 2)
  return distance

def convert_to_binary(num, digits):
  binary = str(bin(num).replace("0b", ""))
  # if the binary number is less than 4 digits, add more zeros
  while len(binary) < digits:
    binary = "0" + binary

  # for the case of the ir sensors, we are reading too many bytes...
  # we are omitting GPA3 because it doesnt read for some reason
  #if len(binary) > digits:
  #  binary = binary[1:3] + binary[4:7]
  return binary

def get_ir_states(bus):
  num = bus.read_byte_data(DEVICE, GPIOA)
  # 5 ir sensors, want 5 states, thus 5 binary digits
  binary = convert_to_binary(num, 8)
  states = [int(x) for x in binary]
  states = states[2:4] + states[5:8]
  return states

def get_encoder_counts(bus):
  # read status of each ir sensor connected to the bus in the form of a list of
  # ints
  num = bus.read_byte_data(DEVICE, GPIOB)
  # request 4 readings
  binary = convert_to_binary(num, 4)
  status = [int(x) for x in binary]
  return status

def thread_function_wheel_velocities():
  global vels
  global prev_counters
  lock_vels = Lock()
  # thread function that gets current wheel velocities every 0.5 seconds
  N = 20
  time_period = 0.5
  while True:
    time.sleep(time_period)
    w = list()
    for i,j in zip(counters, prev_counters):
      n = i - j
      speed = (2*math.pi*n) / (N*time_period)
      if speed > 255:
        speed = 255
      w.append(speed)
    
    lock_vels.acquire()
    vels = w
    prev_counters = counters
    lock_vels.release()


def thread_function_distance(bus):
  global dist
  lock_dist = Lock()
  while True:
    # modify global variable
    distance = get_distance(bus)
    if distance > 255 or distance < 0:
      distance = 0
    lock_dist.acquire()
    dist = distance
    lock_dist.release()

def thread_function_encoder(bus):
  global counters
  global prev_counters
  lock_counters = Lock()
  last_status = get_encoder_counts(bus)
  while True:
    encoder_status = get_encoder_counts(bus)
    lock_counters.acquire()
    changes = [a^b for a,b in zip(encoder_status, last_status)]
    counters = [a+b for a,b in zip(counters, changes)]
    lock_counters.release()
    last_status = encoder_status

def shutdown_sensors(bus, x, y, z, signum, frame):
  bus.write_byte_data(DEVICE, OLATA, 0)
  bus.write_byte_data(DEVICE, OLATB, 0)
  x.join()
  y.join()
  z.join()
 
if __name__ == '__main__':
  bus = bus_setup()
  # create a thread that gets distance measurements every 2 seconds,
  # returns a valid number in cm or returns an error,
  # which includes a bad number (negative, really large, etc) or a number that
  # is too far away to really use
  x = threading.Thread(target=thread_function_distance, args=(bus,))
  x.start()
  y = threading.Thread(target=thread_function_encoder, args=(bus,))
  y.start()
  z = threading.Thread(target=thread_function_wheel_velocities)
  z.start()
  # if we are shutting down, set everything to 0 and join the threads
  signal.signal(signal.SIGTERM, partial(shutdown_sensors, bus, x, y, z))

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    # wait until ros node is setup
    while True:
      try:
        s.connect((HOST, PORT))
        break
      except:
        continue

    while True:
      try:
        # get the ir sensor states: 0 means detected something, 1 means not
        # detected
        ir_states = get_ir_states(bus) 
        time.sleep(.25)
        sensors_send = ir_states + [vels[3], vels[2], vels[1], vels[0]]
        sensors_send.append(int(dist))
        # put sensor measurement list into a format to send over TCP connection
        sensors_send_bytes = pickle.dumps(sensors_send)
        s.sendall(sensors_send_bytes)
      except KeyboardInterrupt:
        print("gets here")
        # set all pins to LOW on bus
        bus.write_byte_data(DEVICE, OLATA, 0)
        bus.write_byte_data(DEVICE, OLATB, 0)
        x.join()
        y.join()
        break


