# Specify minimum CMake version for ROS 2 compatibility
cmake_minimum_required(VERSION 3.10)

# Project name
project(cca_ros_action)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)

# Enable stricter warnings for GCC and Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # ROS 2 communication library
find_package(rclcpp_action REQUIRED)
find_package(cca_ros REQUIRED)
find_package(cca_ros_util REQUIRED)
find_package(cca_ros_viz_msgs REQUIRED)  # To plan and visualize joint and EE trajectories

# Include directories for headers
include_directories(
  include  # Include directory for this package
  ${rclcpp_INCLUDE_DIRS}  # Include rclcpp headers
  ${Eigen3_INCLUDE_DIRS}  # Include Eigen3 headers
)

# Create a library target for this package
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp  # Source file for the library
)

# Specify ROS dependencies
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp rclcpp_action cca_ros cca_ros_util cca_ros_viz_msgs)

# Add executable
add_library(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp 
)

# Link non-ROS libraries to executable
target_link_libraries(${PROJECT_NAME}_node PUBLIC ${PROJECT_NAME}) 

# Specify ROS dependencies
ament_target_dependencies(${PROJECT_NAME}_node PUBLIC rclcpp)

# Export dependencies for downstream ROS packages
ament_export_dependencies(rclcpp_action cca_ros cca_ros_util cca_ros_viz_msgs)

# Export the library target for visibility to other ROS packages
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Install include directory
install(
	DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Install the library and its headers
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME} # Export for visibility with other CMake packages
  ARCHIVE DESTINATION lib  # Static library installation path
  LIBRARY DESTINATION lib  # Dynamic library installation path
  RUNTIME DESTINATION bin  # Executable installation path
  INCLUDES DESTINATION include # Header files
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Configure testing (if enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Find linting tools
  ament_lint_auto_find_test_dependencies()  # Find test dependencies
endif()

# Register package with ament so package is visible in the ROS system
ament_package()
