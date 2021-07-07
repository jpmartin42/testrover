cmake_minimum_required(VERSION 3.0.2)
project(testrover_description)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  tf
  urdf
  xacro
  joint_state_publisher
  joint_state_publisher_gui
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
