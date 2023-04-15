cmake_minimum_required(VERSION 3.0.2)
project(lorett)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  genmsg
  rospy
  std_msgs
  message_generation
  sensor_msgs
  geographic_msgs
  image_transport
  cv_bridge
)

add_message_files(
  FILES
  sdr_log.msg
)

add_service_files(
  FILES
  Sdr_recorder_ros.srv
  Slaid_show.srv
  Get_pass_ros.srv
  sdr_calibtrate.srv
  sdr_recorder.srv
)


generate_messages(
  DEPENDENCIES
  std_msgs
)






include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/sdr_python3.py src/slaid_show.py src/get_pass.py src/auto_sdr.py src/SDRReader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)