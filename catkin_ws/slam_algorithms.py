#!/usr/bin/env python3
# *** slam_algorithms.py ***
# input: robot state and current map
# output: robot new velocities

import numpy as np
import time
import math

# calculate the distance between the two measurements
def distance(p1, p2):
  return math.sqrt((p1[0]-p2[0])**2 + (p1[0]-p2[0])**2) 


def bug0(q, c_points):
  # if q is within 10 cm of any collision point, turn counter clockwise
  # otherwise, keep going forward
  collision = False
  for point in c_points:
    if distance(q, point) < 10:
      collision = True
  # if there is a collision, increase angular velocity
  # if there isn't, increase forward velocity
  if collision:
    return [0.0,0.0,0.1]
  else:
    return [0.0, 0.1, 0.0]

