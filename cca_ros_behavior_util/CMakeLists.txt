cmake_minimum_required(VERSION 3.8)
project(cca_ros_behavior_util)

# Set C++ standard to 20
set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cca_ros REQUIRED)
find_package(cca_ros_util REQUIRED)

# Install the 'include' directory
install(DIRECTORY include/
  DESTINATION include
)

# Export dependencies for visibility to other downstream ROS packages
ament_export_include_directories(include)
ament_export_dependencies(behaviortree_cpp cca_ros cca_ros_util)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
