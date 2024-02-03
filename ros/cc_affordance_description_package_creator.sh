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
package_name="cc_affordance_${robot_name}_description"

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
cat << EOF > $package_name/launch/cc_affordance_${robot_name}_description.launch.py
"""
Use this launch file to load all parameters and run all robot-related nodes for CC Affordance Planning.

Three things need to be done in this file:
1. Load cc_affordance_<robot>_ros_setup.yaml file onto the ROS parameter server.
2. Launch robot_description onto the ROS parameter server.
3. Run robot_state_publisher.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()

    # Load cc_affordance_<robot>_ros_setup.yaml onto the ROS parameter server
    #---------------------------------------------------------------#
    ld.add_action(
        DeclareLaunchArgument(
            'cc_affordance_robot_ros_setup',
            default_value=os.path.join(get_package_share_directory('cc_affordance_${robot_name}_description'), 'config', 'cc_affordance_${robot}_ros_setup.yaml'),
            description='Path to the YAML file containing the robot-related ROS setup for the CC Affordance work'
        )
    )

    param_loader_node = Node(
        package='rosparam',
        executable='rosparam',
        arguments=['load', LaunchConfiguration('cc_affordance_robot_ros_setup')],
        output='screen',
    )
    ld.add_action(param_loader_node)

    # Load robot description onto the ROS parameter server here
    #---------------------------------------------------------------#

    # Run robot state publisher here
    #---------------------------------------------------------------#

    return ld
EOF


# Notify user
echo "Package created successfully!"
echo "Package name: $package_name"
echo "YAML files created in $package_name/config"
echo "Launch file created in $package_name/launch"
