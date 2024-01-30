# Specify minimum CMake version for ROS 2 compatibility
cmake_minimum_required(VERSION 3.10)

# Project name
project(moveit_plan_and_viz)

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
find_package(rosidl_default_generators REQUIRED) # For message generation
find_package(control_msgs REQUIRED)  # ROS 2 messages for robot control
find_package(geometry_msgs REQUIRED)  # ROS 2 messages for geometry
find_package(visualization_msgs REQUIRED) # To use visualization markers in Rviz
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# Include directories for headers
include_directories(
  ${rclcpp_INCLUDE_DIRS}  # Include rclcpp headers
  ${Eigen3_INCLUDE_DIRS}  # Include Eigen3 headers
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveItPlanAndViz.srv"
  DEPENDENCIES
  trajectory_msgs
)

# Create executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp moveit_core moveit_ros_planning moveit_visual_tools )

# To use interfaces in the target from within this package
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executables
install(TARGETS
   ${PROJECT_NAME}_node
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  # launch
  DESTINATION share/${PROJECT_NAME}/
)

# Configure testing (if enabled)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)  # Find linting tools
  ament_lint_auto_find_test_dependencies()  # Find test dependencies
endif()

# Register package with ament so package is visible in the ROS system
ament_package()
