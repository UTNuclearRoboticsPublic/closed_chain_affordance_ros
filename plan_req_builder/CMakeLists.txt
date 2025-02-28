cmake_minimum_required(VERSION 3.8)
project(plan_req_builder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(cca_ros REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(interactive_goal_interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)


# Non-ROS packages
find_package(Eigen3 REQUIRED)
# Find non-ROS libraries manually
FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)
FIND_LIBRARY(cc_affordance_planner_LIBRARIES cc_affordance_planner /usr/local/lib)

# Include directories for headers
include_directories(
  include  # Include directory for this package
  ${rclcpp_INCLUDE_DIRS}  # Include rclcpp headers
  ${Eigen3_INCLUDE_DIRS}  # Include Eigen3 headers
)

# Specify the node executables
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

# Specify ROS dependencies for the target
ament_target_dependencies(${PROJECT_NAME} rclcpp interactive_markers rclcpp_action cca_ros interactive_goal_interfaces tf2_eigen)

# Link Eigen libraries against this project library
target_link_libraries(${PROJECT_NAME} affordance_util::affordance_util cc_affordance_planner::cc_affordance_planner Eigen3::Eigen)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()