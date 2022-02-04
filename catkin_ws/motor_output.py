#!/usr/bin/env python3
# *** motor_output.py ***
# Implements PID control on the mobile robot's wheel velocities
# input: wheel velocities
# output: wheel veloities changed by PID controller
import socket
import rospy
import time
import pickle
import numpy as np
from std_msgs.msg import Float32MultiArray
from functools import partial

HOST = socket.gethostbyname('ubuntu_lgpio-motor_1')
PORT = 81

class PID:
  def __init__(self):
    self.last3errors = np.array([[0.0,0.0,0.0,0.0],
                                 [0.0,0.0,0.0,0.0],
                                 [0.0,0.0,0.0,0.0]])
    self.error = np.array([0.0,0.0,0.0,0.0])
    self.kp = 1
    self.ki = 1
    self.kd = 1
    self.t_period = 0.1
    self.prev_vels = np.array([0.0,0.0,0.0,0.0])
    self.new_vels = np.array([0.0,0.0,0.0,0.0])


  def control(self, vels):
    vels = [round(i,4) for i in vels]
    # do PID control to output a new velocity
    # convert the vels (list) into an numpy array for calculation
    ref_vels = np.asarray(vels)
    # get the error between the current velocity and the desired
    self.error = np.subtract(ref_vels,self.prev_vels)
    # get the change in error from the last error measurement to the current one
    #error_change = np.abs(np.subtract(self.error,self.last3errors[0,:]))
    self.new_vels = np.add(self.prev_vels,self.kp*self.error)
    # round to 2 decimal places
    return self

  def get_new_vels(self):
    new_vels = self.new_vels.tolist()
    new_vels = [round(i,4) for i in new_vels]
    return new_vels

  def step(self):
    # go to the next step
    self.prev_vels = self.new_vels
    self.last3errors[2,:] = self.last3errors[1,:]
    self.last3errors[1,:] = self.last3errors[0,:]
    self.last3errors[0,:] = self.error



def handle_velocities(data, args):
  s = args[0]
  pid = args[1]
  # s: the file descriptor for writing to the socket that is 
  #    connected to drive_motors.py
  pid.control(data.data)
  send_bytes = pickle.dumps(pid.get_new_vels())
  s.sendall(send_bytes)
  # set new vels to prev vels, push errors down the list, 
  pid.step()

# ros subscriber that takes in a list of wheel velocities from map.py
def init_node(s):
  pid = PID()
  rospy.init_node('velocity_listener', anonymous=True)
  rospy.Subscriber("velocity_chatter", Float32MultiArray, handle_velocities, (s, pid))
  rospy.spin()

if __name__ == '__main__':
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    init_node(s)

