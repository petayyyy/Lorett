cmake_minimum_required(VERSION 3.0.2)
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

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_service_files(
  FILES
  takeoff.srv
  publishPose.srv
  yawPose.srv

  GetTelemetry.srv
  Navigate.srv
  NavigateGlobal.srv
  SetPosition.srv
  SetVelocity.srv
  SetAttitude.srv
  SetRates.srv
  SetLEDEffect.srv
  Execute.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

add_executable(arming src/arming.cpp)
target_link_libraries(arming ${catkin_LIBRARIES})

add_executable(telemetry src/telemetry.cpp)
target_link_libraries(telemetry ${catkin_LIBRARIES})

add_executable(simple_offboard src/simple_offboard.cpp)
target_link_libraries(simple_offboard
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES})

add_executable(vpe_publisher src/vpe_publisher.cpp)
target_link_libraries(vpe_publisher ${catkin_LIBRARIES})

install(TARGETS simple_offboard  vpe_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})