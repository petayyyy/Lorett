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
  dynamic_reconfigure
  geometry_msgs
  sensor_msgs
  geographic_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
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

set(SOURCES src/arming.cpp include/lorett_c4s/takeoff.h include/lorett_c4s/publishPose.h include/lorett_c4s/yawPose.h)
add_executable(arming ${SOURCES})
target_link_libraries(arming ${catkin_LIBRARIES})

add_executable(telemetry src/telemetry.cpp)
target_link_libraries(telemetry ${catkin_LIBRARIES})

set(SOURCES2 src/simple_offboard.cpp include/lorett_c4s/Navigate.h include/lorett_c4s/NavigateGlobal.h include/lorett_c4s/SetPosition.h include/lorett_c4s/GetTelemetry.h include/lorett_c4s/SetVelocity.h include/lorett_c4s/SetAttitude.h include/lorett_c4s/SetRates.h)
add_executable(simple_offboard ${SOURCES2})
target_link_libraries(simple_offboard
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES})

add_executable(vpe_publisher src/vpe_publisher.cpp)
target_link_libraries(vpe_publisher ${catkin_LIBRARIES})

install(TARGETS simple_offboard  vpe_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})