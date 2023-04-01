cmake_minimum_required(VERSION 3.0.2)
project(fork_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  forkmotion.msg
  forkposition.msg
  agvmotion.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs 
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
  std_msgs
  actionlib_msgs
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
