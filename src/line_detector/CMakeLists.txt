cmake_minimum_required(VERSION 2.8.3)
project(line_detector)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  cv_bridge
  image_transport
  tf
)

find_package(OpenCV 3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/line_detector
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/line_detector.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}



add_executable(${PROJECT_NAME}_node src/line_detector_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  line_detector
)
