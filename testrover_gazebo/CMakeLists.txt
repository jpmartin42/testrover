cmake_minimum_required(VERSION 3.0.2)
project(testrover_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
  gazebo_ros
  gazebo_ros_control
  gazebo_plugins
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
