#!/usr/bin/env python3
# *** sensors_recv.py ***
# purpose: take in data from sensors.py, send it to a ros node that maps out
# environment
import socket
import time
import rospy
import pickle
from robot.msg import sensorData

HOST = socket.gethostbyname('ubuntu_ros_1')    # localhost
#HOST = '0.0.0.0'    # localhost
PORT = 80          # port to listen on

def setup_publisher():
  pub = rospy.Publisher('sensor_chatter', sensorData, queue_size=10)
  rospy.init_node('sensor_talker', anonymous=True)
  r = rospy.Rate(4)  # publishes 10 times per second
  return pub, r

def publish(data, pub, r):
  msg = sensorData()
  msg.ir_states = data[0:5]
  msg.velA = data[5]
  msg.velB = data[6]
  msg.velC = data[7]
  msg.velD = data[8]
  msg.distance = data[9]

  pub.publish(msg)
  #r.sleep()



if __name__ == '__main__':
  pub,r = setup_publisher()
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

        
