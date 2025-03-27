cmake_minimum_required(VERSION 3.8)
project(cca_ros_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)  # For actions if you use them

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
  "msg/PlanningRequest.msg"
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
  ${srv_files}
  ${action_files}
  DEPENDENCIES
  trajectory_msgs
  geometry_msgs
  actionlib_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
