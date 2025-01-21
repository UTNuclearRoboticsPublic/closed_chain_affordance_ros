cmake_minimum_required(VERSION 3.8)
project(cca_interactive_goals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)
FIND_LIBRARY(cc_affordance_planner_LIBRARIES cc_affordance_planner /usr/local/lib)

set(CMAKE_AUTOMOC ON)

# rviz_ui_panel
include_directories(include)

set(HEADERS
  include/${PROJECT_NAME}/panel.hpp
)

set(SOURCES
  src/panel.cpp
)

qt5_wrap_ui(UIC_FILES
  src/ui/panel.ui
)

add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
  ${HEADERS}
  ${UIC_FILES}
)

target_link_libraries(${PROJECT_NAME}
  rviz_common::rviz_common
  Qt5::Widgets
  ${rviz_default_plugins_LIBRARIES}
  ${rviz_common_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering
  rviz_default_plugins
  interactive_markers
  tf2
  tf2_geometry_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  rviz
  src/ui
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
)

ament_package()
