cmake_minimum_required(VERSION 2.8.3)
project(pid)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  tf
  nav_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   pid_errores.msg
)

add_message_files(FILES pid_errores.msg)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pid
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

add_executable(pid_main src/pid_main.cpp)
target_link_libraries(pid_main ${catkin_LIBRARIES})
add_dependencies(pid_main pid_generate_messages_cpp)

add_executable(pid_main_retardos src/pid_main_retardos.cpp)
target_link_libraries(pid_main_retardos ${catkin_LIBRARIES})
add_dependencies(pid_main_retardos pid_generate_messages_cpp)




