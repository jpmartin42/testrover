cmake_minimum_required(VERSION 3.0.2)
project(testrover_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  gazebo_ros
  gazebo_ros_control
  gazebo_plugins
)

find_package(gazebo REQUIRED)

catkin_package(
  LIBRARIES
    rover_gazebo_diff_plugin
  CATKIN_DEPENDS 
    roscpp
    gazebo_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Reference custom plugin
add_library(rover_gazebo_diff_plugin src/diff_suspension_plugin.cpp)
target_link_libraries(rover_gazebo_diff_plugin ${GAZEBO_LIBRARIES})

install(
  TARGETS rover_gazebo_diff_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)