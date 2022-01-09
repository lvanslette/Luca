#!/usr/bin/env python3
# *** drive_motors.py ***
# -> input: wheel velocities, output pwms/directions of wheels to the motors 
import lgpio as lg
from functools import partial
import signal
import time
import socket
import pickle

HOST = socket.gethostbyname('ubuntu_lgpio-motor_1')
PORT = 81
# GPIO pins associated with the motor drivers
pwm_pins = [20, 6, 23, 24]   # fL, fR, bL, bR
fwd_pins = [19, 18, 12, 5]
bwd_pins = [4, 16, 22, 25]

# rate at which you should control the speed of the motors
#FREQ = 100
FREQ = 250

def gpio_setup():
  h = lg.gpiochip_open(0)
  # only for testing, this needs to be removed when we get real data
  for i,j in zip(fwd_pins,bwd_pins):
    lg.gpio_claim_output(h, i)
    lg.gpio_claim_output(h, j)

  return h

def shutdown_motors(h, signal, frame):
  for i,j,k in zip(fwd_pins,bwd_pins,pwm_pins):
    lg.gpio_write(h, i, 0)
    lg.gpio_write(h, j, 0)
    lg.tx_pwm(h, k, FREQ, 0) 
  lg.gpiochip_close(h)

 

# converts wheel velocities from motor_output.py into direction and pwm values 
def vel2pwm(wheel_velocities):
  directions = [1,1,1,1]
  # calculate directions: returns a 1 if forward, -1 if reverse
  for i in range(len(wheel_velocities)):
    vel = wheel_velocities[i]
    # if the velocity is 0, directions should default to 1
    if vel != 0:
      # TODO: add pwms[i] = 0 here
      directions[i] = int(abs(vel)/vel)

  # calculate pwms
  wheel_speeds = [abs(vel) for vel in wheel_velocities]
  new_pwms = [int(300*speed) for speed in wheel_speeds]
  #new_pwms = [19,16,18,18]
  #new_pwms = [0,0,0,0]
  return directions, new_pwms

if __name__ == '__main__':
  h = gpio_setup()
  signal.signal(signal.SIGTERM, partial(shutdown_motors, h))
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
      print('connected by', addr)
      while True:
        data = conn.recv(1024)
        if not data:
          break
        wheel_velocities = pickle.loads(data)
        # convert wheel velocities into wheel directions and pwms
        directions, new_pwms = vel2pwm(wheel_velocities)
        #print("wheel directions: ", directions)
        #print("wheel pwms: ", new_pwms)

        # adjust motor direction for each wheel,
        # adjust pwm values for each wheel  
        for i,j,k,d,pwm in zip(pwm_pins,fwd_pins,bwd_pins,directions,new_pwms):
          lg.gpio_write(h, j, (d+1)//2)
          lg.gpio_write(h, k, (d-1)//-2)
          lg.tx_pwm(h, i, FREQ, pwm)
      


