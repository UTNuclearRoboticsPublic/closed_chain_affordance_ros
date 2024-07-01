# Specify minimum CMake version for ROS 2 compatibility
cmake_minimum_required(VERSION 3.10)

# Project name
project(affordance_util_ros)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)

# Enable stricter warnings for GCC and Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and link required packages
find_package(Eigen3 REQUIRED)  # Linear algebra library
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # ROS 2 communication library
find_package(tf2_ros REQUIRED)  # ROS 2 library for managing coordinate frames
find_package(control_msgs REQUIRED)  # ROS 2 messages for robot control
find_package(geometry_msgs REQUIRED)  # ROS 2 messages for geometry
find_package(sensor_msgs REQUIRED)  # ROS 2 messages for sensor data

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

# Link Eigen libraries against this project library
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Specify ROS dependencies
ament_target_dependencies(${PROJECT_NAME} rclcpp tf2_ros control_msgs geometry_msgs sensor_msgs)

# Export the library target for visibility to other ROS packages
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

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


# Configure testing (if enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Find linting tools
  ament_lint_auto_find_test_dependencies()  # Find test dependencies
endif()

# Register package with ament so package is visible in the ROS system
ament_package()
