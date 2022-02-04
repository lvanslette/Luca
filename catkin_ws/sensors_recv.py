#!/usr/bin/env python3
# *** sensors_recv.py ***
# purpose: take in data from sensors.py, send it to a ros node that maps out
# environment
import socket
import time
import rospy
import pickle
import threading
from threading import Thread
from robot.msg import sensorData
from std_msgs.msg import Float32MultiArray

HOST = socket.gethostbyname('ubuntu_ros_1')    # localhost
#HOST = '0.0.0.0'    # localhost
PORT = 80          # port to listen on

next_directions = [1,1,1,1]

def setup_publisher():
  pub = rospy.Publisher('sensor_chatter', sensorData, queue_size=10)
  rospy.init_node('sensor_talker', anonymous=True)
  r = rospy.Rate(4)  # publishes 10 times per second
  return pub, r

def setup_subscriber():
  rospy.Subscriber('velocity_chatter', Float32MultiArray, save_directions)
  rospy.spin()

def save_directions(data):
  global next_directions
  # if velocity is 0, return direction as 0
  next_directions = [i//abs(i) if i else 0 for i in data.data]

def publish(data, pub, r):
  msg = sensorData()
  msg.ir_states = data[0:5]
  msg.velA = next_directions[0]*data[5]
  msg.velB = next_directions[1]*data[6]
  msg.velC = next_directions[2]*data[7]
  msg.velD = next_directions[3]*data[8]
  # if the length of the array is less than 10, we don't have a new distance measurement
  if len(data) > 9:
    msg.distance = data[9]
  else:
    # if no new message
    msg.distance = -1.0

  #print(msg.distance)
  pub.publish(msg)
  #r.sleep()



if __name__ == '__main__':
  pub,r = setup_publisher()
  # setup subscriber thread to get the newest wheel velocity directions from map.py
  x = threading.Thread(target=setup_subscriber)
  x.start()
  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    with conn:
      print('sensors_recv: connected by', addr)
      while not rospy.is_shutdown():
        try:
          data = pickle.loads(conn.recv(1024))
        except:
          break
        # data format (all in ints):
        #  [ 0 0 0 0 0    0 0 0 0         0 ]
        #   -ir sensors- -velocity-- -distance sensor-

        # publish data in the sensorData format
        publish(data, pub, r)
      

        
