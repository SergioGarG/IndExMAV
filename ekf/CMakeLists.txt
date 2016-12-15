cmake_minimum_required(VERSION 2.8.3)
project(ekf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  tf
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   filter_state.msg
   Navdata.msg
   mensaje_kalman.msg
   mensaje_kalman_array.msg
   Ardrone3PilotingStateAltitudeChanged.msg
   Ardrone3PilotingStateAttitudeChanged.msg
   Ardrone3PilotingStateSpeedChanged.msg
   vector31.msg
   vector21.msg
   navdata_demo.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_message_files(FILES Navdata.msg)
add_message_files(FILES mensaje_kalman.msg)
add_message_files(FILES mensaje_kalman_array.msg)
add_message_files(FILES Ardrone3PilotingStateAltitudeChanged.msg)
add_message_files(FILES Ardrone3PilotingStateAttitudeChanged.msg)
add_message_files(FILES Ardrone3PilotingStateSpeedChanged.msg)
add_message_files(FILES navdata_altitude.msg)
add_message_files(FILES vector31.msg)
add_message_files(FILES vector21.msg)
add_message_files(FILES navdata_demo.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kalman
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ekf src/ekf.cpp)
target_link_libraries(ekf ${catkin_LIBRARIES})
add_dependencies(ekf ekf_generate_messages_cpp)

add_executable(ekf_wo_delay src/ekf_wo_delay.cpp)
target_link_libraries(ekf_wo_delay ${catkin_LIBRARIES})
add_dependencies(ekf_wo_delay ekf_generate_messages_cpp)





