cmake_minimum_required(VERSION 3.8)
project(cca_ik_comp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)  # Linear algebra library
find_package(yaml-cpp REQUIRED ) # To parse yaml files

find_package(moveit_ros_planning REQUIRED)
find_package(affordance_util_ros REQUIRED)
find_package(moveit_plan_and_viz REQUIRED)
FIND_LIBRARY(affordance_util_LIBRARIES affordance_util /usr/local/lib)

# Include directories for headers
include_directories(
  include  # Include directory for this package
  ${rclcpp_INCLUDE_DIRS}  # Include rclcpp headers
  ${Eigen3_INCLUDE_DIRS}  # Include Eigen3 headers
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp moveit_ros_planning affordance_util_ros moveit_plan_and_viz)
target_link_libraries(${PROJECT_NAME}_node affordance_util yaml-cpp Eigen3::Eigen
)

# Install the executables
install(TARGETS
   ${PROJECT_NAME}_node
   DESTINATION lib/${PROJECT_NAME}
 )

install(DIRECTORY
  # launch
  DESTINATION share/${PROJECT_NAME}/
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
