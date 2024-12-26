cmake_minimum_required(VERSION 3.8)
project(cca_ros_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(cca_ros REQUIRED)

# Specify include directories
include_directories(
  include
)

# Add libraries
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/${PROJECT_NAME}.cpp)

# Specify ROS dependencies for the library target
ament_target_dependencies(${PROJECT_NAME} rclcpp behaviortree_cpp cca_ros)

# Install the 'include' directory
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Export the library target for visibility to other ROS packages
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

# Install the library and its headers
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME} # Export for visibility with other CMake packages
  ARCHIVE DESTINATION lib  # Static library installation path
  LIBRARY DESTINATION lib  # Dynamic library installation path
  RUNTIME DESTINATION bin  # Executable installation path
  INCLUDES DESTINATION include # Header files
)

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
