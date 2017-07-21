cmake_minimum_required(VERSION 2.8.3)
project(ackermann_cpp_rewrite)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ackermann_msgs
  geometry_msgs
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ackermann_cpp_rewrite_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
