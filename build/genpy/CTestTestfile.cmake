# CMake generated Testfile for 
# Source directory: /home/pi/ros_catkin_ws/src/genpy
# Build directory: /home/pi/ros_catkin_ws/build/genpy
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_genpy_nosetests_test "/home/pi/ros_catkin_ws/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/home/pi/ros_catkin_ws/src/catkin/cmake/test/run_tests.py" "/home/pi/ros_catkin_ws/build/test_results/genpy/nosetests-test.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/pi/ros_catkin_ws/build/test_results/genpy" "/usr/bin/nosetests-2.7 -P --process-timeout=60 --where=/home/pi/ros_catkin_ws/src/genpy/test --with-xunit --xunit-file=/home/pi/ros_catkin_ws/build/test_results/genpy/nosetests-test.xml")
subdirs("scripts")
