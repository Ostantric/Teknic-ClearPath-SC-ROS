cmake_minimum_required(VERSION 3.1.0)
project(joystick_bridge)

find_package(catkin REQUIRED COMPONENTS
  joy
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(xbox_controller src/joystick.cpp)
target_link_libraries(xbox_controller ${catkin_LIBRARIES})

