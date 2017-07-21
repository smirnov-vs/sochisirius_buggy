cmake_minimum_required(VERSION 2.8.3)
project(joycontrol)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  joy
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(teleop_joy src/teleop_joy.cpp)
target_link_libraries(teleop_joy ${catkin_LIBRARIES})
