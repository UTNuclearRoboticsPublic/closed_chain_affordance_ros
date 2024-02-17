#!/bin/bash

# ========================================================================
# Script Name: cc_affordance_description_package_creator.sh
# Description: Automates the creation of a ROS2 cca_<robot> package,
#              generating templates for essential configuration
#              files in the format required for CC Affordance planning.
# Author: Crasun Jans
# ========================================================================

# Check if ROS2 is sourced
if [[ -z "$ROS_DISTRO" ]]; then  # Check if the ROS_DISTRO environment variable is not set
  echo "Please source ROS2 before running this script."
  exit 1
fi

# Get robot name
read -p "Enter robot name: " robot_name

# Construct package name
package_name="cca_${robot_name}"

# Create package
ros2 pkg create $package_name --build-type ament_cmake --dependencies rclcpp cc_affordance_planner_ros

# Create config and launch folders
mkdir $package_name/config
mkdir $package_name/launch

# Remove unnecessary folders
rm -rf $package_name/include

# Create the description file
cat << EOF > $package_name/config/cca_${robot_name}_description.yaml
# **Description of the ${robot_name} robot
# Define the reference frame, joint axes, their locations, and the tool's position.
# Add or remove joint fields as needed to accurately represent the robot
ref_frame:
  - name: # Reference frame name, example: arm0_base_link

joints:
  - name: # Joint name, example: arm0_shoulder_yaw
    w: # Joint axis, example: [0, 0, 1]
    q: # Joint location, example: [0, 0, 0]

  - name:
    w:
    q:

  - name:
    w:
    q:

  - name:
    w:
    q:

  - name:
    w:
    q:

  - name:
    w:
    q:

tool:
  - name: # Tool frame name, example: arm0_tool0. This is usually at the center of the palm
    q: # Tool frame location, example: [0.9383, 0.0005, 0.0664]
EOF

# Create the ROS setup file
cat << EOF > $package_name/config/cca_${robot_name}_ros_setup.yaml
# *** ROS-related attributes pertaining to ${robot_name} *** #

cc_affordance_planner_ros:
  ros__parameters:
    # --- Robot Name ---
    cca_robot: "${robot_name}" # This package must be named cca_<cca_robot>

    # --- Action Server ---
    cca_robot_as: "" # Follow joint trajectory action server to execute trajectory on the robot

    # --- Topics ---
    cca_joint_states_topic: "" # Joint states topic name

    # --- Robot Description ---
    cca_robot_description_parameter: "" # Robot description parameter name

    # --- Planning Group ---
    cca_planning_group: "" # MoveIt planning group name
EOF

# Create the launch file
cat << EOF > $package_name/launch/cca_${robot_name}.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Define LaunchConfiguration arguments with clear descriptions
    cca_${robot_name}_ros_setup = os.path.join(
        get_package_share_directory('cca_${robot_name}'),
        'config',
        'cca_${robot_name}_ros_setup.yaml'
        )

    # Create a Node instance for the cc_affordance_spot_description node
    cc_affordance_planner_ros_node_with_params = Node(
        package="cca_${robot_name}",
        executable="cc_${robot_name}_node",
        name="cc_affordance_planner_ros",
        emulate_tty=True,
        output='screen',
        prefix='xterm -e', # to receive input from user
        parameters=[
     cca_${robot_name}_ros_setup
        ],
    )
    ld.add_action(cc_affordance_planner_ros_node_with_params)
    return ld
EOF

# Create the affordance planner src file
cat << EOF > $package_name/src/cca_${robot_name}_node.cpp
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcAffordancePlannerRos>("cc_affordance_planner_ros", node_options);

    // Start spinning the node in a separate thread so we could do things like reading parameters and joint states
    // inside the node
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });
    /*------------------------------------------------------------*/
    // Specify affordance screw
    const Eigen::Vector3d aff_screw_axis(0, 0, 1);          // screw axis
    const Eigen::Vector3d aff_screw_axis_location(0, 0, 0); // location vector
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Set affordance goal
    const double aff_goal = 0.5 * M_PI; // Code
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Optionally set planner parameters
    /* const double &aff_step = 0.3; */
    /* const int &gripper_control_par_tau = 1; */
    /* const double &accuracy = 10.0 / 100.0; */
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Run the planner
    bool success = node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal);
    // Or if getting affordance screw location from apriltag
    /* const std::string apriltag_frame_name = "affordance_frame"; */
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal); Note screw axis is
     * manually set in this case as aff_screw_axis above. Just the location is gotten from Apriltag*/
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Optionally, with planner parameters call:
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal, aff_step,
     * gripper_control_par_tau, accuracy); */
    // or
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal, aff_step,
     * gripper_control_par_tau, accuracy); */
    /*------------------------------------------------------------*/

    // Call the function again with new (or old) aff_screw_axis, aff_screw_axis_location and aff_goal to execute
    // another affordance in series
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal); */
    // or
    /*bool success = node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal); */
    if (success) // If trajectory was successfully executed wait until, goal response callback is invoked and ros is
                 // shutdown from there
    {
        spinner_thread.join(); // join the spinning thread and exit
    }
    else
    {
        rclcpp::shutdown();    // shutdown ROS on failure
        spinner_thread.join(); // join the spinner thread
    }

    return 0;
}
EOF

# Create the CMakeLists file
cat << EOF > $package_name/src/cca_${robot_name}_node.cpp
cmake_minimum_required(VERSION 3.8)
project(cca_${robot_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find and link required packages
find_package(Eigen3 REQUIRED)  # Linear algebra library
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cc_affordance_planner_ros REQUIRED)

# Specify the node executables
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# Specify ROS dependencies for the target
ament_target_dependencies(${PROJECT_NAME}_node rclcpp cc_affordance_planner_ros)

# Link Eigen libraries against this project library
target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
EOF

# Notify user
echo "Package created successfully!"
echo "Package name: $package_name"
echo "CMakeLists.txt created in $package_name"
echo "YAML files created in $package_name/config"
echo "Launch file created in $package_name/launch"
echo "Cpp file created in $package_name/src"
