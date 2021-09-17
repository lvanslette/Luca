#!/usr/bin/env python3
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import RPi.GPIO as gp
import wiringpi as wpi
import numpy as np
import time
import os
# when this import is called, returns a bunch of warnings
import AdapterBoard
  
def publish_message():
  # Node is publishing to the video_frames topic using 
  # the message type Image
  pub = rospy.Publisher('video_frames', Image, queue_size=10)
     
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name.
  rospy.init_node('video_pub_py', anonymous=True)
     
  # Go through the loop 10 times per second
  rate = rospy.Rate(10) # 10hz

  # create AdapterBoard object that takes care of taking care of camera images
  #   goal: publish one of the camera images continuously 
  adapter_board = AdapterBoard.MultiAdapter()
  adapter_board.init(320, 240)
  #adapter_board.preview()

  # Create a VideoCapture object
  # The argument '0' gets the default webcam.
  # ERROR ON THIS LINE: DOESNT LIKE THIS CALL
  #cap = cv2.VideoCapture(-1)

     
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # While ROS is still running.
  while not rospy.is_shutdown():
     
      # Capture frame-by-frame
      # This method returns True/False as well
      # as the video frame.
      #ret, frame = cap.read()
      # get a snapshot from camera A. camera A = 0, camera B = 1 
      ret, frame = adapter_board.get_snapshot(0)
      # get a snapshot from camera B (camera B located on adapter board C port, thus 0x06)
      #ret, frame = adapter_board.get_snapshot("0x06")
         
      if ret == True:
        # Print debugging information to the terminal
        rospy.loginfo('publishing video frame')
             
        # Publish the image.
        # The 'cv2_to_imgmsg' method converts an OpenCV
        # image to a ROS image message
        pub.publish(br.cv2_to_imgmsg(frame))
             
      # Sleep just enough to maintain the desired rate
      rate.sleep()
         
if __name__ == '__main__':
  try:
    publish_message()
  except rospy.ROSInterruptException:
    pass
