# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/usr/include".split(';') if "${prefix}/include;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "cpp_common;rostime".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lrosconsole;-lrosconsole_log4cxx;-lrosconsole_backend_interface;/usr/lib/arm-linux-gnueabihf/liblog4cxx.so;/usr/lib/arm-linux-gnueabihf/libboost_regex.so;/usr/lib/arm-linux-gnueabihf/libboost_system.so;/usr/lib/arm-linux-gnueabihf/libboost_thread.so;/usr/lib/arm-linux-gnueabihf/libboost_chrono.so;/usr/lib/arm-linux-gnueabihf/libboost_date_time.so;/usr/lib/arm-linux-gnueabihf/libboost_atomic.so;/usr/lib/arm-linux-gnueabihf/libpthread.so".split(';') if "-lrosconsole;-lrosconsole_log4cxx;-lrosconsole_backend_interface;/usr/lib/arm-linux-gnueabihf/liblog4cxx.so;/usr/lib/arm-linux-gnueabihf/libboost_regex.so;/usr/lib/arm-linux-gnueabihf/libboost_system.so;/usr/lib/arm-linux-gnueabihf/libboost_thread.so;/usr/lib/arm-linux-gnueabihf/libboost_chrono.so;/usr/lib/arm-linux-gnueabihf/libboost_date_time.so;/usr/lib/arm-linux-gnueabihf/libboost_atomic.so;/usr/lib/arm-linux-gnueabihf/libpthread.so" != "" else []
PROJECT_NAME = "rosconsole"
PROJECT_SPACE_DIR = "/opt/ros/kinetic"
PROJECT_VERSION = "1.12.17"
