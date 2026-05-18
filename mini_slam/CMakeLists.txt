cmake_minimum_required(VERSION 3.8)
project(mini_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(mini_slam_core
  src/scan_processor.cpp
)
target_include_directories(mini_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mini_slam_core
  builtin_interfaces
  nav_msgs
  sensor_msgs
)

add_executable(mini_slam_node
  src/mini_slam_node.cpp
)
target_link_libraries(mini_slam_node
  mini_slam_core
)
ament_target_dependencies(mini_slam_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mini_slam_core mini_slam_node
  EXPORT export_mini_slam_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_mini_slam_core HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  nav_msgs
  sensor_msgs
)

ament_package()
