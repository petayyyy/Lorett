cmake_minimum_required(VERSION 3.0)

project(lorett_c4s)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  genmsg
  std_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  geographic_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)
add_service_files(
  FILES
  takeoff.srv
  publishPose.srv
  yawPose.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(arming src/arming.cpp)
target_link_libraries(arming ${catkin_LIBRARIES})

add_executable(telemetry src/telemetry.cpp)
target_link_libraries(telemetry ${catkin_LIBRARIES})

