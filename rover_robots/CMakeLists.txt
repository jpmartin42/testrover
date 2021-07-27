cmake_minimum_required(VERSION 3.0.2)
project(rover_robots)

find_package(catkin REQUIRED COMPONENTS
  roslaunch

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
