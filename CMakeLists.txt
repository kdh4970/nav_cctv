cmake_minimum_required(VERSION 3.0.2)
project(nav_cctv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  dynamic_reconfigure
  std_msgs
  message_generation
)

add_message_files(FILES Locations.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES nav_cctv
  CATKIN_DEPENDS roscpp std_msgs std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_executable(point_publisher src/point_publisher.cpp)
target_link_libraries(point_publisher ${catkin_LIBRARIES})
add_dependencies(point_publisher nav_cctv_gencpp)

add_library(simple_layer src/simple_layer.cpp)
add_library(grid_layer src/grid_layer.cpp)
add_library(dynamic_layer src/dynamic_layer.cpp)


