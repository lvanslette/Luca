#!/usr/bin/env python3
# *** map.py ***
# input: sensor data from sensors_recv.py (through ros node)
# output: broadcast sensor data (to VM) and send output of wheel velocities
#         to motor_output.py (through ros node)
# -> this program determines the next best location to go to 
#     eventually will do path planning (local and global)
#     will also do localization, mapping with kalman filtering
#from slam_algorithms import bug0
import rospy
import math
import signal
import socket
import pickle
import numpy as np
import time
from robot.msg import sensorData
from std_msgs.msg import Float32MultiArray, String

class Map:
  def __init__(self):
    self.ir_rotation_matrix = [[-0.2,0.0],[-0.1,0.2],[0.1,0.2],[0.2,0.0],[0.0,-0.2]]
    # q = [x, y, theta]
    self.q = [0,0,0]
    self.q_prev = [0,0,0]
    self.t_period = 0.25   
    self.collision_points = []
    self.prev_distance = []
    rospy.on_shutdown(self.on_shutdown)

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
    w_vels = [round(vel,2) for vel in [msg.velA, msg.velB, msg.velC, msg.velD]]
    self.get_body_vels(w_vels)
    # using new body vels, get new q
    self.q = [round(i+(j*self.t_period),2) for i,j in zip(self.q_prev, self.body_vels)]
    # get new collision points
    self.new_cPoints = list()
    self.new_cPoints.extend(self.handle_ir(list(msg.ir_states)))
    self.new_cPoints.append(self.handle_distance(msg.distance))
    # get rid of duplicate points, add the unique points to the list 
    # of previously found collision points
    self.add_new_points()
    # get new wheel vels using the new data received from the sensors
    self.get_new_wheel_velocities()
    # send new data to the display map and to the robot
    self.publish_data()

    self.q_prev = self.q

    return self

  def get_body_vels(self, w_vels):
    # vels = [vel motor A, B, C, D] of each wheel as a float
    # output: q_dot = [x,y,theta]
    H_0_pinv = np.array([[-1/0.48, 1/0.48, 1/0.48, -1/0.48],
                         [ 1/4,    1/4,    1/4,     1/4],
                         [-1/4,    1/4,   -1/4,     1/4]])
    u = np.array(w_vels)
    #u = np.array([w_vels[3], w_vels[2], w_vels[0], w_vels[1]])
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
    if self.prev_distance != distance and distance > 0:
      c_point = self.q[0:2]
      c_point[1] += 0.12 + distance
    self.prev_distance = distance
    return c_point

  def add_new_points(self):
    for c_point in self.new_cPoints:
      # if there is more than one collision point with the same values, don't add it
      # also don't add empty points
      if self.collision_points.count(c_point) < 1 and c_point != []:
        self.collision_points.append(c_point)
      else: 
        self.new_cPoints.remove(c_point)
    return self

  def get_new_wheel_velocities(self):
    # using the new collision points, current position, and current body velocity,
    # calculate new wheel velocities

    self.new_body_velocities = self.calc_new_vels()
    self.new_wheel_velocities = Float32MultiArray()
    self.new_wheel_velocities.data = self.body2wheel()
    return self

  # TODO
  def calc_new_vels(self):
    # using bug 0
    # -> propagate to next expected position using current velocities
    # -> if next position is close enough to a collision point, turn CCW away from it
    # -> otherwise, continue forward (aka to your goal position)
    # NEED TO MODIFY TO ACCOUNT FOR ANGULAR VELOCITY (SLIPPAGE AFFECTING X,Y)
    q_predict = [i+(j*self.t_period) for i,j in zip(self.q, self.body_vels)]
    # placeholder: if I'm within 50cm of an object in my next state, stop going forward and rotate
    # FROM slam_algorithms.py
    return self.bug0(q_predict[0:2], self.collision_points)
    #return [0.0,0.0,0.0]


  def bug0(self, q, c_points):
    collision = False
    for point in c_points:
      proximity = round(math.sqrt((q[0]-point[0])**2 + (q[1]-point[1])**2), 2)
      # if the collision point is within 30 cm of the next state
      if proximity < 0.15:
        collision = True
    if collision:
      print("going to collide!")
      return [0.0,-0.1,0.01]
      #return [0.0,0.0,0.0]
    else:
      return [0.0,0.1,0.0]
      #return [0.0,0.0,0.0]
 

  def body2wheel(self):
    H_0 = np.array([[-0.12, 1, -1],
                    [ 0.12, 1,  1],
                    [ 0.12, 1, -1],
                    [-0.12, 1,  1]])
    return np.dot(H_0, self.new_body_velocities)


  def publish_data(self):
    # send to motor_output.py
    self.publish_to_motor.publish(self.new_wheel_velocities)
    # send to display_map.py
    #print("x,y:", self.q[0], self.q[1])
    #print("orientation:", self.q[2])
    display_bytes = pickle.dumps([self.q[0],self.q[1],self.q[2],self.new_cPoints])
    self.publish_to_map[0].sendto(display_bytes, self.publish_to_map[1])


if __name__ == '__main__':
  Map()

