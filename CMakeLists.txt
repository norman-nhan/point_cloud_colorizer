cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_colorizer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  message_filters
  cv_bridge
  pcl_ros
)

find_package(OpenCV REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  # CATKIN_DEPENDS roscpp sensor_msgs message_filters cv_bridge pcl_ros
)

## Build the node
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node 
  src/point_cloud_colorizer_node.cpp
  src/PointCloudColorizer.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)