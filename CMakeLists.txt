SET(CMAKE_CXX_FLAGS "-std=c++11")
cmake_minimum_required(VERSION 3.5.1)
project(meteor_forklift)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} /include
)

add_library(stm32_controller_lib src/stm32_controller.cpp)

add_library(ros_comminication_lib src/ros_comminication.cpp)


add_executable(forklift src/main.cpp)
target_link_libraries(forklift ros_comminication_lib stm32_controller_lib ${catkin_LIBRARIES})
