#!/usr/bin/env python3
# *** display_map.py ***
# takes in the input sent over the socket by map.py and displays it as
# a map

import matplotlib
import numpy as np
# matplotlib is build on numpy arrays and designed to work with SciPy
import matplotlib.pyplot as plt
import socket
import pickle
import time

local_IP = '192.168.1.201'
local_PORT = 8888
bufferSize = 4096
# create UDP socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# bind UDP socket to IP and port
UDPServerSocket.bind((local_IP, local_PORT))
print("UDP server listening...")


def update_graph(data, plot, ax, fig, xLim, yLim):
  # data format: first point is current location
  #              second point is all collision points
  q = np.array([data[0], data[1]])
  # add the robot position to the plot initially...
  #   that way the robot position will always be the first thing plotted
  #   -> this method could be a way of plotting the robot with a different
  #      color and shape compared to the collision points
  # q must give an Nx2 numpy array

  # split collision points into their x and components
  collision_points = data[2]

  # get the current points as a numpy array with shape (N,2)
  array = q
  for point in collision_points:
    point = np.asarray(point)
    # if an empty point is returned
    if len(point) == 2:
        # add each collision point as an additional ROW in the np array
        array = np.vstack([array, point])
  # plot we added the robot position above, now we add the collision points to
  # the plot
  plot.set_offsets(array)
  
  # xLim, yLim are passed by reference, so modifying them in the fn works fine
  xLim[0] = -5-np.amin(array, axis=0)[0]
  xLim[1] = 5+np.amax(array, axis=0)[0]
  yLim[0] = -5-np.amin(array, axis=0)[1]
  yLim[1] = 5+np.amax(array, axis=0)[1]
  # change the axes so that the robot is always in the center
  ax.set_xlim(xLim[0], xLim[1])
  ax.set_ylim(yLim[0], yLim[1])

  # update the graph
  fig.canvas.draw()

if __name__ == '__main__':
  # turns on interactive mode: controls if the figure is redrawn for every
  #   draw() command. without ion() enabled, the figure wouldn't update
  #   itself
  plt.ion()
  # subplot(): recommeneded way of creating a single figure and axes
  #   -> subplots(2) would result in 2 graphs in a single figure
  fig, ax = plt.subplots()   
  # creates empty scatter plot
  plot = ax.scatter([0],[0], color="blue", s=10)   
  # store a variable for the current maximum/minimum values
  xLim = [-5,5]
  yLim = [-5,5]
  # sets axes of scatter plot
  ax.set_xlim(xLim[0], xLim[1])
  ax.set_ylim(yLim[0], yLim[1])

  while True:
    # hangs until received message from host
    data_hex, addr = UDPServerSocket.recvfrom(bufferSize)
    # message comes in as a byte-address pair, separate the two
    data = pickle.loads(data_hex)
    # plot the points
    update_graph(data, plot, ax, fig, xLim, yLim)
