cmake_minimum_required(VERSION 3.10)

project(cca_ros_rviz_plugin)

# Set C++ standard to 20 for modern C++ support
set(CMAKE_CXX_STANDARD 20)

# Enable stricter warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cca_ros REQUIRED)
find_package(cca_ros_msgs REQUIRED)
find_package(cca_ros_action REQUIRED)
find_package(ros_cpp_util REQUIRED)

# Non-ROS Packages
# External libraries used for visualization and linear algebra
find_package(Eigen3 REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(cc_affordance_planner REQUIRED)

set(CMAKE_AUTOMOC ON) # Enable Qt's automatic moc generation

# Path to include headers
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  include
)

# Headers and source files for the plugin and marker manager
set(HEADERS
	include/${PROJECT_NAME}/cca_ros_rviz_plugin.hpp
	include/${PROJECT_NAME}/interactive_marker_manager.hpp
)

set(SOURCES
	src/${PROJECT_NAME}/cca_ros_rviz_plugin.cpp
	src/${PROJECT_NAME}/interactive_marker_manager.cpp
)

# Build the shared library for the plugin
add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
  ${HEADERS}
)

# Link libraries against this package's library
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  interactive_markers
  tf2_ros
  cca_ros
  cca_ros_msgs
  cca_ros_action
  ros_cpp_util
)

# Link required libraries
target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
  ${rviz_default_plugins_LIBRARIES}
  ${rviz_common_LIBRARIES}
  cc_affordance_planner
  Eigen3::Eigen
)

# Install libraries
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install include headers
install(DIRECTORY
  include/
  DESTINATION include
)

# Enable automated linting and tests if BUILD_TESTING is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export plugin description for RViz
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  pluginlib
  interactive_markers
  tf2_ros
  cca_ros
  cca_ros_msgs
  cca_ros_action
  ros_cpp_util
)

# Finalize package definition
ament_package()
