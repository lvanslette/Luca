#!/usr/bin/env python3
# *** map.py ***
# input: sensor data from sensors_recv.py (through ros node)
# output: broadcast sensor data (to VM) and send output of wheel velocities
#         to motor_output.py (through ros node)
# -> this program determines the next best location to go to 
#     eventually will do path planning (local and global)
#     will also do localization, mapping with kalman filtering
#from slam_algorithms import bug0
import logging
# change logging levels
logging.basicConfig(filename="test.log", level=logging.DEBUG)
# log info into a file, not to the console
import rospy
from math import sqrt, cos, sin, tan, atan2, pi
import signal
import socket
import pickle
import numpy as np
import time
from robot.msg import sensorData
from std_msgs.msg import Float32MultiArray, String

from slam_algorithms import bug0

sim = True

class Map:
  def __init__(self):
    #self.robot = Robot()
    self.ir_rotation_matrix = [[0.0,0.2],[0.2,0.1],[0.2,-0.1],[0.0,-0.2],[-0.2,0.0]]
    # q = [x, y, theta]
    self.q = [0,0,0]
    self.q_prev = [0,0.0,0.0]
    self.t_period = 0.1   
    self.collision_points = []
    self.prev_distance = []
    self.new_wheel_velocities = Float32MultiArray()
    self.new_wheel_velocities.data = [0.001,0.001,0.001,0.001]
    rospy.on_shutdown(self.on_shutdown)

    self.setup_connections()
    self.listen_for_measurements()

  def on_shutdown(self):
    logging.info("shutting down bitches")

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
    if sim:
      rospy.Subscriber('sensor_chatter', sensorData, self.sim_handle_msg)
    else:
      rospy.Subscriber('sensor_chatter', sensorData, self.handle_msg)
    rospy.spin()

  def sim_handle_msg(self, msg):
    # SIM ENVIRONMENT: takes no sensor data, 
    w_vels = self.new_wheel_velocities.data
    self.wheel2body(w_vels)
    self.body2world(self.body_vels)
    self.propagate_q()
    #print("body_vels:", self.body_vels)
    #print("s_vels:", self.s_vels)
    self.new_cPoints = self.sim_cPoints()
    self.add_new_points()
    self.get_new_wheel_velocities()
    self.publish_data()
    self.q_prev = self.q

  def sim_cPoints(self):
    # add a collision point that is directly straight ahead, colliding with sandbox edge
    new_cPoints = list()
    # for distance sensor, add a collision point that exists directly forward
    # for ir sensors, add points if they are within a small range of the robot

    new_cPoints.append(self.sim_distance())
    return new_cPoints

  def sim_distance(self):
    q = self.q
    # check for parallel lines
    is_parallel = False
    for ang in [0, pi/2, pi, 3*pi/2, 2*pi]:
      if ang == q[2]:
        is_parallel = True
    if is_parallel:
      return self.parallel()
    #if any([q[2] == i for i in []):
    #  return self.parallel()
    # rectangle dimensions
    r_h = .88
    r_w = .78
    # determine slope
    m = tan(q[2])
    # determine y intercept
    b = q[1]-m*q[0]
    # check if the intersection of each rectangle side is valid
    points = self.find_valid_sides(m, b)
    # from the 2 correct sides, based on the orientation, return the correct point
    point = self.find_side(points, q[2])
    return point

  def parallel(self):
    if self.q[2] == 0:
      return [.39, self.q[1]]
    if self.q[2] == pi/2:
      return [self.q[0], .44]
    if self.q[2] == pi:
      return [-.39, self.q[1]]
    if self.q[2] == 3*pi/2:
      return [self.q[0], -.44]
    if self.q[2] == 2*pi:
      return [.39, self.q[1]]

  def find_valid_sides(self, m, b):
    points = list()
    # for right vertical
    cPoint = [.39, m*.39+b]
    if cPoint[1] <= .44 and cPoint[1] >= -.44:
      points.append(cPoint)
    # for left vertical
    cPoint = [-.39, m*-.39+b]
    if cPoint[1] <= .44 and cPoint[1] >= -.44:
      points.append(cPoint)
    # for top horizontal
    cPoint = [(.44-b)/m, .44]
    if cPoint[0] <= .39 and cPoint[0] >= -.39:
      points.append(cPoint)
    # for bottom horizontal
    cPoint = [(-.44-b)/m, -.44]
    if cPoint[0] <= .39 and cPoint[0] >= -.39:
      points.append(cPoint)

    return points

  def find_side(self, points, theta):
    # calculate distance
    angs = list()
    for p in points:
      x = p[0]-self.q[0]
      y = p[1]-self.q[1]
      ang = atan2(y,x)
      if ang < 0:
        ang = 2*pi+ang
      if ang > 2*pi:
        ang = ang%(2*pi)
      angs.append(ang)

    if abs(angs[0]-theta) < abs(angs[1]-theta):
      return points[0]
    else:
      return points[1]


  def handle_msg(self, msg):
    # shouldn't the velocities get changed at the end, using prev velocities 
    # to measure current sensors?
    # msg format: msg = [velA, velB, velC, velD, distance, ir_states]
    w_vels = [round(vel,4) for vel in [msg.velA, msg.velB, msg.velC, msg.velD]]
    # input wheel velocities, output corresponding world (x,y,theta) velocities
    self.wheel2body(w_vels)
    self.body2world(self.body_vels)
    # using new body vels, get new q
    self.propagate_q()
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

  def propagate_q(self):
    # if we are using this method for bug 0
    # update state q with measured body velocities
    ang_x = self.t_period*cos(self.q_prev[2])
    ang_y = self.t_period*sin(self.q_prev[2])
    new_q = [0,0,0]
    new_q[0] = self.q_prev[0]+self.s_vels[0]*self.t_period
    new_q[1] = self.q_prev[1]+self.s_vels[1]*self.t_period
    new_q[2] = self.q_prev[2]+self.s_vels[2]*self.t_period
    if new_q[2] > 2*pi:
      new_q[2] = new_q[2]%(2*pi)
    self.q = new_q
    return new_q


  def wheel2body(self, w_vels):
    # WHEEL TO BODY VELOCITY
    # vels = [vel motor A, B, C, D] of each wheel as a float
    # output: q_dot = [x_dot,y_dot,theta_dot]
    H_0_pinv = (1/.03)*np.array([[ 1/4,   1/4,  1/4,  1/4],
                                 [-1/4,   1/4,  1/4, -1/4],
                                 [   0, -25/6, 25/6,    0]])

    u = np.array(w_vels)
    # x represents forward, y represents to the left
    self.body_vels = np.dot(H_0_pinv, u)
    return self
  
  def body2world(self, b_vels):
    # BODY TO WORLD VELOCITY
    # using current x,y,theta and body velocity just calculated,
    # determine what the x_dot,y_dot,theta_dot would be 
    theta = self.q[2]
    self.R_b2s = np.array([[ cos(theta), -1*sin(theta), 0],
                           [ sin(theta),    cos(theta), 0],
                           [          0,             0, 1]])
    self.s_vels = [round(vel, 5) for vel in np.dot(self.R_b2s, np.array(b_vels))]
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
    if self.prev_distance != distance and distance > 0:
      c_point = self.q[0:2]
      c_point[0] += (0.12 + distance)*cos(self.q[2])
      c_point[1] += (0.12 + distance)*sin(self.q[2])
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
    self.new_wheel_velocities.data = self.body2wheel()
    return self

  def calc_new_vels(self):
    # using bug 0
    # -> propagate to next expected position using current velocities
    # -> if next position is close enough to a collision point, turn CCW away from it
    # -> otherwise, continue forward (aka to your goal position)
    # NEED TO MODIFY TO ACCOUNT FOR ANGULAR VELOCITY (SLIPPAGE AFFECTING X,Y)

    q_predict = self.propagate_q()
    return self.bug0(q_predict[0:2], self.collision_points)
    #return bug0(q_predict[0:2], self.collision_points)
    #return [0.0,0.0,0.0]

  
  def bug0(self, q, c_points):
    collision = False
    for point in c_points:
      proximity = round(sqrt((q[0]-point[0])**2 + (q[1]-point[1])**2), 2)
      # if the collision point is within 30 cm of the next state
      if proximity < 0.15:
        collision = True
    if collision:
      # bug 0, turn right until can go forward
      #self.q[2] += .1
      print("collision")
      return [-0.0001,0.000,0.001]
      #return [0.0,0.0,0.0]
      #return [0.0,0.0,0.01]
    else:
      return [0.0001,0.000,0.0]
      #return [0.0,0.0,0.0]
  
  def body2wheel(self):
    # WORLD TO BODY VELOCITY
    #self.R_s2b = np.linalg.inv(self.R_b2s)
    #self.new_body_velocities = np.dot(R_s2b, self.new_world_velocities) 

    # BODY TO WHEEL VELOCITY
    H_0 = (1/.03)*np.array([[1, -1,  0],
                            [1,  1, -0.12],
                            [1,  1,  0.12],
                            [1, -1,  0]])
    return np.dot(H_0, self.new_body_velocities)


  def publish_data(self):
    # send to motor_output.py
    if not sim:
      self.publish_to_motor.publish(self.new_wheel_velocities)
    # send to display_map.py
    display_bytes = pickle.dumps([self.q[0],self.q[1],self.q[2],self.new_cPoints])
    self.publish_to_map[0].sendto(display_bytes, self.publish_to_map[1])

"""
class Robot:
  def __init__(self):
    self.w_vels
    self.body_vels
""" 

if __name__ == '__main__':
  Map()

