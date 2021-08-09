cmake_minimum_required(VERSION 3.0.2)
project(rover_hw)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  hardware_interface

)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    rover_hw
  CATKIN_DEPENDS
    roscpp
    std_msgs
    hardware_interface
  DEPENDS
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/robot_joint_interface.hpp
  include/${PROJECT_NAME}/abstract_robot.hpp
  src/abstract_robot.cpp
  src/robot_joint_interface.cpp
)

# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install( 
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)