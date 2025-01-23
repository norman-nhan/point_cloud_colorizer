cmake_minimum_required(VERSION 3.5)
project(point_cloud_colorizer_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(${PROJECT_NAME}_node
  src/PointCloudColorizer.cpp
  src/point_cloud_colorizer_node.cpp)

target_include_directories(${PROJECT_NAME}_node
  PRIVATE
    include
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  message_filters
  pcl_ros
  cv_bridge
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
