cmake_minimum_required(VERSION 3.0.2)
project(nav_cctv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  costmap_2d
  dynamic_reconfigure
)

catkin_package(

)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_library(simple_layer src/simple_layer.cpp)
add_library(grid_layer src/grid_layer.cpp)


