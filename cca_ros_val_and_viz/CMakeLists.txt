cmake_minimum_required(VERSION 3.10)

project(cca_ros_val_and_viz)

# Set C++ standard to 20 for modern C++ support
set(CMAKE_CXX_STANDARD 20)

# Enable stricter warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS Packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(cca_ros_msgs REQUIRED)
find_package(ros_cpp_util REQUIRED)

# Create the main executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp moveit_core moveit_ros_planning moveit_visual_tools cca_ros_msgs ros_cpp_util)

# Install the executable
install(TARGETS
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install additional resources
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Enable automated linting and tests if BUILD_TESTING is enabled
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize package definition
ament_package()
