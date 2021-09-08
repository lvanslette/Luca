# Install script for directory: /home/pi/ros_catkin_ws/src/genmsg

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/pi/ros_catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/pi/ros_catkin_ws/build/genmsg/catkin_generated/installspace/genmsg.pc")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genmsg/cmake" TYPE FILE FILES "/home/pi/ros_catkin_ws/build/genmsg/catkin_generated/installspace/genmsg-extras.cmake")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genmsg/cmake" TYPE FILE FILES
    "/home/pi/ros_catkin_ws/build/genmsg/catkin_generated/installspace/genmsgConfig.cmake"
    "/home/pi/ros_catkin_ws/build/genmsg/catkin_generated/installspace/genmsgConfig-version.cmake"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genmsg" TYPE FILE FILES "/home/pi/ros_catkin_ws/src/genmsg/package.xml")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/genmsg/cmake" TYPE FILE FILES
    "/home/pi/ros_catkin_ws/src/genmsg/cmake/pkg-genmsg.cmake.em"
    "/home/pi/ros_catkin_ws/src/genmsg/cmake/pkg-genmsg.context.in"
    "/home/pi/ros_catkin_ws/src/genmsg/cmake/pkg-msg-extras.cmake.in"
    "/home/pi/ros_catkin_ws/src/genmsg/cmake/pkg-msg-paths.cmake.develspace.in"
    "/home/pi/ros_catkin_ws/src/genmsg/cmake/pkg-msg-paths.cmake.installspace.in"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/pi/ros_catkin_ws/build/genmsg/catkin_generated/safe_execute_install.cmake")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/genmsg" TYPE PROGRAM FILES "/home/pi/ros_catkin_ws/src/genmsg/scripts/genmsg_check_deps.py")
endif()

