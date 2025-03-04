cmake_minimum_required(VERSION 3.8)
project(cca_ros_viz_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs)
find_package(geometry_msgs)

set(msg_files
  "msg/Goal.msg"
  "msg/GripperGoalType.msg"
  "msg/KinematicState.msg"
  "msg/MotionType.msg"
  "msg/PlannerConfig.msg"
  "msg/ScrewInfo.msg"
  "msg/ScrewLocationMethod.msg"
  "msg/ScrewType.msg"
  "msg/TaskDescription.msg"
  "msg/UpdateMethod.msg"
  "msg/VirtualScrewOrder.msg"
)

set(srv_files
  "srv/CcaRosViz.srv"
)

set(action_files
  "action/CcaRosAction.action"
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${action_files}
)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES
  trajectory_msgs
  geometry_msgs
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
