cmake_minimum_required(VERSION 2.8.3)
project(keyboard_teleop)

find_package(catkin REQUIRED COMPONENTS
  keyboard
  mavros_msgs
  geometry_msgs
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(keyboard_teleop src/Keyboard_teleop.cpp src/main.cpp)
target_link_libraries (keyboard_teleop ${catkin_LIBRARIES})
