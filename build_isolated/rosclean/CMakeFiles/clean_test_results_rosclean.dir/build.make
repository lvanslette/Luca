# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/ros_catkin_ws/src/ros/rosclean

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/ros_catkin_ws/build_isolated/rosclean

# Utility rule file for clean_test_results_rosclean.

# Include the progress variables for this target.
include CMakeFiles/clean_test_results_rosclean.dir/progress.make

CMakeFiles/clean_test_results_rosclean:
	/usr/bin/python2 /opt/ros/kinetic/share/catkin/cmake/test/remove_test_results.py /home/pi/ros_catkin_ws/build_isolated/rosclean/test_results/rosclean

clean_test_results_rosclean: CMakeFiles/clean_test_results_rosclean
clean_test_results_rosclean: CMakeFiles/clean_test_results_rosclean.dir/build.make

.PHONY : clean_test_results_rosclean

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results_rosclean.dir/build: clean_test_results_rosclean

.PHONY : CMakeFiles/clean_test_results_rosclean.dir/build

CMakeFiles/clean_test_results_rosclean.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_rosclean.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results_rosclean.dir/clean

CMakeFiles/clean_test_results_rosclean.dir/depend:
	cd /home/pi/ros_catkin_ws/build_isolated/rosclean && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/ros_catkin_ws/src/ros/rosclean /home/pi/ros_catkin_ws/src/ros/rosclean /home/pi/ros_catkin_ws/build_isolated/rosclean /home/pi/ros_catkin_ws/build_isolated/rosclean /home/pi/ros_catkin_ws/build_isolated/rosclean/CMakeFiles/clean_test_results_rosclean.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clean_test_results_rosclean.dir/depend

