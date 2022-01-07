#!/usr/bin/env python3
# *** map.py ***
# input: sensor data from sensors_recv.py (through ros node)
# output: broadcast sensor data (to VM) and send output of wheel velocities
#         to motor_output.py (through ros node)
# -> this program determines the next best location to go to 
#     eventually will do path planning (local and global)
#     will also do localization, mapping with kalman filtering
import rospy
import signal
import socket
import pickle
import numpy as np
import time
from robot.msg import sensorData
from std_msgs.msg import Float32MultiArray, String

class Map:
  def __init__(self):
    self.ir_rotation_matrix = [[-1,0],[-1,1],[1,1],[1,0],[0,-1]]
    self.q = [0,0,0]
    self.q_prev = [0,0,0]
    self.t_period = 0.25   
    self.collision_points = []
    self.prev_distance = []
    rospy.on_shutdown(self.on_shutdown)
    #rospy.signal_shutdown("finished")

    self.setup_connections()
    self.listen_for_measurements()

  def on_shutdown(self):
    print("shutting down bitches")

  def setup_connections(self):
    # setup UDP connection between map.py and display_map.py
    #   display_map.py is running on the local computer, map.py is runnning in the RasPi
    display_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    serverAddressPort = ("192.168.1.201",8888)
    self.publish_to_map = [display_socket, serverAddressPort]

    # setup ROS connection between map.py and motor_output.py
    rospy.init_node('velocity_talker', anonymous=True)
    self.publish_to_motor = rospy.Publisher('velocity_chatter', Float32MultiArray, queue_size=10)
    return self

  def listen_for_measurements(self):
    # waits for sensors_recv.py to publish new sensor data from the robot
    rospy.Subscriber('sensor_chatter', sensorData, self.handle_msg)
    rospy.spin()

  def handle_msg(self, msg):
    # msg format: msg = [velA, velB, velC, velD, distance, ir_states]
    self.get_body_vels([msg.velA, msg.velB, msg.velC, msg.velD])
    # using new body vels, get new q
    self.q = [i+(j*self.t_period) for i,j in zip(self.q_prev, self.body_vels)]
    # get new collision points
    self.collision_points.extend(self.handle_ir(list(msg.ir_states)))
    self.collision_points.append(self.handle_distance(msg.distance))
    # remove duplicate collision points
    self.remove_duplicates()

    # get new wheel vels using the new data received from the sensors
    self.get_new_wheel_velocities()
    # send new data to the display map and to the robot
    self.publish_data()

    self.q_prev = self.q

    return self

  def get_body_vels(self, w_vels):
    # vels = [vel motor A, B, C, D] of each wheel as a float
    # output: q_dot = [x,y,theta]
    H_0_pinv = np.array([[-1/48, 1/48, 1/48, -1/48],
                       [ 1/4,  1/4,  1/4,   1/4],
                       [-1/4,  1/4, -1/4,   1/4]])
    u = np.array([w_vels[3], w_vels[2], w_vels[0], w_vels[1]])
    self.body_vels = np.dot(H_0_pinv, u)
    return self

  def handle_ir(self, ir_states):
    # input: [_ _ _ _ _]
    # output: for each ir sensor, if ir state is a 0, add 
    #         collision point at (current position + some stuff)
    q = self.q
    collisions = list()
    for index in range(len(ir_states)):
      state = ir_states[index]
      if state == 0:
        collision_point = [0,0]
        collision_point[0] += q[0] + self.ir_rotation_matrix[index][0] 
        collision_point[1] += q[1] + self.ir_rotation_matrix[index][1] 
        collisions.append(collision_point) 
    return collisions

  def handle_distance(self, distance):
    # if we have a new distance measurement, convert it to a collision point
    c_point = []
    # if the change in distance is greater than 0.01, add it
    if self.prev_distance != distance:
      c_point = self.q[0:2]
      c_point[1] += round(0.12 + distance/100, 2)
    self.prev_distance = distance
    return c_point

  def remove_duplicates(self):
    for c_point in self.collision_points:
      # if there is more than one collision point with the same values, remove it
      if self.collision_points.count(c_point) > 1:
        self.collision_points.remove(c_point)
    return self


  def get_new_wheel_velocities(self):
    # using the new collision points, current position, and current body velocity,
    # calculate new wheel velocities

    self.new_body_velocities = self.calc_new_vels()
    # self.new_body_velocities = [0.0, 0.0, 0.0]
    self.new_wheel_velocities = Float32MultiArray()
    self.new_wheel_velocities.data = self.body2wheel()
    return self

  def calc_new_vels(self):
    # use potential field local path planning to map out an environment

    return [0.0,0.0,0.0]
    
  def body2wheel(self):
    H_0 = np.array([[-12, 1, -1],
                    [ 12, 1,  1],
                    [ 12, 1, -1],
                    [-12, 1,  1]])
    return np.dot(H_0, self.new_body_velocities)

  
  def publish_data(self):
    # send to motor_output.py
    self.publish_to_motor.publish(self.new_wheel_velocities)
    # send to display_map.py
    display_bytes = pickle.dumps([self.q[0],self.q[1],self.collision_points])
    self.publish_to_map[0].sendto(display_bytes, self.publish_to_map[1])


if __name__ == '__main__':
  Map()

