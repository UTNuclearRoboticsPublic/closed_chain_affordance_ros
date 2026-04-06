cmake_minimum_required(VERSION 3.10)

project(cca_ros_behavior_features)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cca_ros_features REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/${PROJECT_NAME}.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp PUBLIC geometry_msgs PUBLIC cca_ros_features)

ament_export_dependencies(rclcpp cca_ros_features geometry_msgs)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
