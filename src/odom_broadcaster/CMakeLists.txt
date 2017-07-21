cmake_minimum_required(VERSION 2.8.3)
project(odom_broadcaster)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/odom_broadcaster.cpp)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
