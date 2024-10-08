# Specify minimum CMake version for ROS 2 compatibility
cmake_minimum_required(VERSION 3.10)

# Project name
project(cc_affordance_planner_ros)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)

# Enable stricter warnings for GCC and Clang compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and link required packages
find_package(Eigen3 REQUIRED)  # Linear algebra library
find_package(yaml-cpp REQUIRED ) # To parse yaml files
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)  # ROS 2 communication library
find_package(rclcpp_action REQUIRED)
find_package(tf2_ros REQUIRED)  # ROS 2 library for managing coordinate frames
find_package(affordance_util_ros REQUIRED)  # Basic ROS 2 utility functions for affordance work
find_package(moveit_plan_and_viz_msgs REQUIRED)  # To plan and visualize joint and EE trajectories
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

# Find cc affordance cpp libraries
FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)
FIND_LIBRARY(cc_affordance_planner_LIBRARIES cc_affordance_planner /usr/local/lib)

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
ament_target_dependencies(${PROJECT_NAME} rclcpp affordance_util_ros moveit_plan_and_viz_msgs rclcpp_action tf2_ros sensor_msgs control_msgs)

# Export ROS dependencies for downstream ROS packages
ament_export_dependencies(affordance_util_ros moveit_plan_and_viz_msgs rclcpp_action tf2_ros sensor_msgs control_msgs)

# Link the executable against external libraries
target_link_libraries(${PROJECT_NAME} affordance_util cc_affordance_planner yaml-cpp Eigen3::Eigen
)

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
