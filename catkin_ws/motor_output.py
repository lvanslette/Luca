#!/usr/bin/env python3

# *** motor_output.py ***
# input: stuff, output the pwms to drive_motors.py in the other container
import socket
import rospy
import time
import pickle
from std_msgs.msg import Float32MultiArray
from functools import partial

HOST = socket.gethostbyname('ubuntu_lgpio-motor_1')
PORT = 81

def handle_velocities(data, s):
  # s: the file descriptor for writing to the socket that is 
  #    connected to drive_motors.py
  send_bytes = pickle.dumps(data.data)
  s.sendall(send_bytes)

# ros subscriber that takes in a list of velocities from map.py
def init_node(s):
  rospy.init_node('velocity_listener', anonymous=True)
  rospy.Subscriber("velocity_chatter", Float32MultiArray, handle_velocities, s)
  rospy.spin()

if __name__ == '__main__':
  # bytes have to be in the range (0,256) 
  # 0-127: positive velocity (0-127 m/s)
  # 128-255: negative velocity (0-127 m/s)
  test_velocities = [0, 0, 0, 0]
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    init_node(s)

