cmake_minimum_required(VERSION 3.0.2)
project(aruco_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aruco_odom
#  CATKIN_DEPENDS clover roscpp rospy tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS nodes/aruco_tf2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS nodes/odom_and_aruco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

