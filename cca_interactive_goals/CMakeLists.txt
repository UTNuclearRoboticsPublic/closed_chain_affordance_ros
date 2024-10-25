cmake_minimum_required(VERSION 3.8)
project(cca_interactive_goals)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cca_interactive_goals_node src/cca_interactive_goals_node.cpp)
ament_target_dependencies(cca_interactive_goals_node rclcpp interactive_markers)
target_include_directories(cca_interactive_goals_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(cca_interactive_goals_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS cca_interactive_goals_node
  DESTINATION lib/${PROJECT_NAME})

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
