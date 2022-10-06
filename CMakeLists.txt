cmake_minimum_required(VERSION 3.0.2)
project(nav_cctv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  dynamic_reconfigure
  std_msgs
  message_generation
)

add_message_files(FILES Point.msg MultiPoint.msg Locations.msg Coordinate.msg)

generate_messages(DEPENDENCIES std_msgs)
generate_dynamic_reconfigure_options(
  cfg/CctvLayer.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cctv_layer
  CATKIN_DEPENDS roscpp std_msgs costmap_2d dynamic_reconfigure
  DEPENDS system_lib
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)
# make sure configure headers are built before any node using them


add_executable(point_publisher src/point_publisher.cpp)
target_link_libraries(point_publisher ${catkin_LIBRARIES})
add_dependencies(point_publisher nav_cctv_gencpp)

add_executable(multipoint_publisher src/multipoint_publisher.cpp)
target_link_libraries(multipoint_publisher ${catkin_LIBRARIES})
add_dependencies(multipoint_publisher nav_cctv_gencpp)

add_library(simple_layer src/simple_layer.cpp)
add_library(grid_layer src/grid_layer.cpp)
add_library(dynamic_layer src/dynamic_layer.cpp)

add_library(cctv_layer src/cctv_layer.cpp)
target_link_libraries(cctv_layer ${catkin_LIBRARIES})
add_dependencies(cctv_layer ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker nav_cctv_gencpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener nav_cctv_gencpp)




