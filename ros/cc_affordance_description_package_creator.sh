#!/bin/bash

# ========================================================================
# Script Name: cc_affordance_description_package_creator.sh
# Description: Automates the creation of a ROS2 CC Affordance description
#              package, generating templates for essential configuration
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
ros2 pkg create $package_name --build-type ament_cmake

# Create config and launch folders
mkdir $package_name/config
mkdir $package_name/launch

# Remove unnecessary folders
rm -rf $package_name/include
rm -rf $package_name/src

# Create the description file
cat << EOF > $package_name/config/cc_affordance_${robot_name}_description.yaml
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
cat << EOF > $package_name/config/cc_affordance_${robot_name}_ros_setup.yaml
# *** ROS-related attributes pertaining to ${robot_name} *** #

cc_affordance_planner_ros:
  ros__parameters:
    # --- Robot Name ---
    cca_robot: "${robot_name}" # This package must be named cc_affordance_<cca_robot>_description

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
    cc_afforance_spot_ros_setup = os.path.join(
        get_package_share_directory('cc_affordance_spot_description'),
        'config',
        'cc_affordance_spot_ros_setup.yaml'
        )

    # Create a Node instance for the cc_affordance_spot_description node
    cc_affordance_planner_ros_node_with_params = Node(
        package="cc_affordance_spot_description",
        executable="cc_affordance_spot_description_node",
        name="cc_affordance_planner_ros",
        emulate_tty=True,
        output='screen',
        prefix='xterm -e', # to receive input from user
        parameters=[
           cc_afforance_spot_ros_setup
        ],
        # Add node-specific parameters and additional launch actions if needed
    )
    ld.add_action(cc_affordance_planner_ros_node_with_params)
    return ld
EOF


# Notify user
echo "Package created successfully!"
echo "Package name: $package_name"
echo "YAML files created in $package_name/config"
echo "Launch file created in $package_name/launch"
