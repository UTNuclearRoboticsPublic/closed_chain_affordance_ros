#!/bin/bash

# ========================================================================
# Script Name: cca_robot_package_creator.sh
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
ros2 pkg create $package_name --build-type ament_cmake --dependencies rclcpp cca_ros

# Create config and launch folders
mkdir $package_name/config
mkdir $package_name/launch

# Remove unnecessary folders
rm -rf $package_name/include

# Create the description file
cat << EOF > $package_name/config/cca_${robot_name}_description.yaml
# **Description of the ${robot_name} robot
# Define the reference frame, joint axes, their locations, and the tool's position.
# Add or remove joint fields as needed to accurately represent the robot.
ref_frame:
  - name: # Example: arm0_base_link

robot_joints:
  - name: # Example: arm0_shoulder_roll 
    w: # Example: [0, 0, 1]
    q: # Example: [0, 0, 0]

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

end_effector:
  - gripper_joint_name: 
    frame_name: # This will be the parent frame for the tool
    q: 

tool:
  - name: # This is usually at the center of the palm
    offset_from_ee_frame: # Tool location from EE frame
EOF

# Create the ROS setup file
cat << EOF > $package_name/config/cca_${robot_name}_ros_setup.yaml
# *** ROS-related attributes pertaining to ${robot_name} *** #

/**:
  ros__parameters:
    # --- Robot Name ---
    cca_robot: "${robot_name}" # Robot name. This package must be named cca_${robot_name}

    # --- Action Servers - follow_joint_trajectory types---
    cca_robot_as: # To execute joint trajectory on the robot

    # cca_gripper_as: # To execute gripper trajectory on the robot. Goal will be sent simultaneously with the robot trajectory but separately. #Optional

    # cca_robot_and_gripper_as: # To execute robot and gripper trajectory together on the robot. One unified trajectory is sent. #Optional

    # --- Joint states topic ---
    cca_joint_states_topic: # Topic to read joint states from

    # --- How to build the robot ---
    cca_build_robot_from: # Possible values are "yaml" or "urdf"
EOF

# Create the ROS Viz setup file
cat << EOF > $package_name/config/cca_${robot_name}_ros_viz_setup.yaml
# *** ROS-related attributes pertaining to ${robot_name} for visualization of joint trajectories *** #

/**:
  ros__parameters:

    joint_states_topic: # Joint states topic name. Example: "robot_driver/joint_states"

    planning_group: # MoveIt planning group name

    ref_frame: # Default frame where the CCA planning plugin will show the screw interactive marker

    ee_frame: # End-effector frame

    tool_frame: # Frame that follows the screw path for planning purposes

    ee_to_tool_offset: # Location of the tool in the EE frame. Example: [0.07805, 0.0008, -0.01772] 

    rviz_fixed_frame: # Base frame from the urdf
EOF

# Create the task execution launch file
cat << 'EOF' > $package_name/launch/cca_${robot_name}.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue            
from launch_ros.substitutions import FindPackageShare         

def generate_robot_description_content():
    """
    Generates robot description utilizing launch args

    Returns:
        - robot_description_content: The robot description generated from the xacro file.
        - launch_args: List of DeclareLaunchArgument objects used to configure the xacro.

    """
    # Launch args needed to build the robot description 
    launch_args = [
        # Example (uncomment later if needed):
        # DeclareLaunchArgument(
        #     "has_arm",
        #     default_value="false",
        #     description="add arm to the robot if true",
        # ),
    ]

    # Collect arg names from declared launch args
    launch_arg_names = [arg.name for arg in launch_args]

    # Pass launch args as xacro parameters
    xacro_command_args = [
        elem
        for arg_name in launch_arg_names
        for elem in (f"{arg_name}:=", LaunchConfiguration(arg_name))
    ]

    xacro_path = PathJoinSubstitution([
    # Uncomment and/or adjust as needed
    #   FindPackageShare("${robot_name}_description"),
    #   "urdf",
    #   "${robot_name}.urdf.xacro",
    ])

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_path, *xacro_command_args]),
        value_type=str,
    )

    return robot_description_content, launch_args

def generate_launch_description():
    """
    Generate the launch description for the cca_${robot_name} node.
    """
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    ))

    debug = LaunchConfiguration('debug')

    # Extract the CCA ros setup parameters for the robot
    cca_${robot_name}_ros_setup = os.path.join(
        get_package_share_directory('cca_${robot_name}'),
        'config',
        'cca_${robot_name}_ros_setup.yaml'
    )

    # Run xterm in debug mode
    node_prefix = PythonExpression([
        "'xterm -e gdb -ex run --args' if '", debug, "' == 'true' else ''"
    ])

    # Emulate teletypewriter in debug mode to take user input
    emulate_tty = PythonExpression([
        "'", debug, "' == 'true'"
    ])

    # Get robot description and any associated launch args
    robot_description_content, robot_description_launch_args = generate_robot_description_content()
    for arg in robot_description_launch_args:
        ld.add_action(arg)

    # Create a Node instance for the cca_${robot_name} node
    cca_${robot_name}_node_with_params = Node(
        package="cca_${robot_name}",
        executable="cca_${robot_name}_node",
        name="cca_${robot_name}",
        prefix=[node_prefix],
        emulate_tty=emulate_tty,
        output="screen",
        parameters=[cca_${robot_name}_ros_setup, 
        {"robot_description": robot_description_content}, # used if robot is built from urdf for CCA
                    ],
    )

    ld.add_action(cca_${robot_name}_node_with_params)
    return ld
EOF

# In the above file replace ${robot_name} which was read as literal due to 'EOF' with the value of that variable
sed -i "s/\${robot_name}/$robot_name/g" \
    $package_name/launch/cca_${robot_name}.launch.py

# Create the action server launch file
cat << 'EOF' > $package_name/launch/cca_${robot_name}_action_server.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue            
from launch_ros.substitutions import FindPackageShare         

def generate_robot_description_content():
    """
    Generates robot description utilizing launch args

    Returns:
        - robot_description_content: The robot description generated from the xacro file.
        - launch_args: List of DeclareLaunchArgument objects used to configure the xacro.

    """
    # Launch args needed to build the robot description 
    launch_args = [
        # Example (uncomment later if needed):
        # DeclareLaunchArgument(
        #     "has_arm",
        #     default_value="false",
        #     description="add arm to the robot if true",
        # ),
    ]

    # Collect arg names from declared launch args
    launch_arg_names = [arg.name for arg in launch_args]

    # Pass launch args as xacro parameters
    xacro_command_args = [
        elem
        for arg_name in launch_arg_names
        for elem in (f"{arg_name}:=", LaunchConfiguration(arg_name))
    ]

    xacro_path = PathJoinSubstitution([
    # Uncomment and/or adjust as needed
    #   FindPackageShare("${robot_name}_description"),
    #   "urdf",
    #   "${robot_name}.urdf.xacro",
    ])

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_path, *xacro_command_args]),
        value_type=str,
    )

    return robot_description_content, launch_args

def generate_launch_description():
    """
    Generates the launch description for the cca_ros_action_${robot_name} node.
    """
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    ))

    debug = LaunchConfiguration('debug')

    # Extract the CCA ros setup parameters for the robot
    cca_${robot_name}_ros_setup = os.path.join(
        get_package_share_directory('cca_${robot_name}'),
        'config',
        'cca_${robot_name}_ros_setup.yaml'
    )

    # Run xterm in debug mode
    node_prefix = PythonExpression([
        "'xterm -e gdb -ex run --args' if '", debug, "' == 'true' else ''"
    ])

    # Emulate teletypewriter in debug mode to take user input
    emulate_tty = PythonExpression([
        "'", debug, "' == 'true'"
    ])

    # Get robot description and any associated launch args
    robot_description_content, robot_description_launch_args = generate_robot_description_content()
    for arg in robot_description_launch_args:
        ld.add_action(arg)

    # Create a Node instance for the cca_ros_action_${robot_name} node
    cca_ros_action_node_with_params = Node(
        package="cca_ros_action",
        executable="cca_ros_action_node",
        name="cca_ros_action_${robot_name}",
        prefix=[node_prefix],
        emulate_tty=emulate_tty,
        output="screen",
        parameters=[cca_${robot_name}_ros_setup, 
        {"robot_description": robot_description_content}, # used if robot is built from urdf for CCA
                    ],
    )

    ld.add_action(cca_ros_action_node_with_params)
    return ld
EOF

# In the above file replace ${robot_name} which was read as literal due to 'EOF' with the value of that variable
sed -i "s/\${robot_name}/$robot_name/g" \
    $package_name/launch/cca_${robot_name}_action_server.launch.py

# Create the visualization server/RVIZ plugin launch file
cat << 'EOF' > $package_name/launch/cca_${robot_name}_viz.launch.py
"""
Author: Crasun Jans

Description:
This launch script launches the `cca_ros_viz` node with the robot's visualization-related parameters along with the URDF and SRDF data passed as robot_description and robot_description_semantic parameters respectively. It also launches RViz with a predefined configuration file for visualizing the robot.

### Overview:
The script generates three key parameters:
1. `robot_description`: Contains the robot's URDF, defining its physical structure, sensors, and actuators.
2. `robot_description_semantic`: Contains the robot's SRDF, defining its semantic properties, such as joint groups and kinematics for motion planning.
3. `cca_ros_viz_setup_params`: Contains parameters such as joint_states_topic, planning_group, etc. specified in the cca_<robot>_ros_viz_setup.yaml file in the cca_<robot> package.

### Customization:
To adapt this script for a different robot, modify only the following functions:
- `generate_robot_description_content()`: Adjust how the URDF content is extracted for your robot.
- `generate_robot_description_semantic_content()`: Update this function to provide the correct SRDF file for the robot.
- `extract_cca_ros_viz_setup_params()`: Update this function to provide the correct path info to the cca_<robot>_ros_viz_setup.yaml file.
"""
import os

import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)

def generate_robot_description_content():
    """
    Generates robot description utilizing launch args

    Returns:
        - robot_description_content: The robot description generated from the xacro file.
        - launch_args: List of DeclareLaunchArgument objects used to configure the xacro.

    """
    # Launch args needed to build the robot description 
    launch_args = [
        # Example (uncomment later if needed):
        # DeclareLaunchArgument(
        #     "has_arm",
        #     default_value="false",
        #     description="add arm to the robot if true",
        # ),
    ]

    # Collect arg names from declared launch args
    launch_arg_names = [arg.name for arg in launch_args]

    # Pass launch args as xacro parameters
    xacro_command_args = [
        elem
        for arg_name in launch_arg_names
        for elem in (f"{arg_name}:=", LaunchConfiguration(arg_name))
    ]

    xacro_path = PathJoinSubstitution([
    # Uncomment and/or adjust as needed
    #   FindPackageShare("${robot_name}_description"),
    #   "urdf",
    #   "${robot_name}.urdf.xacro",
    ])

    robot_description_content = ParameterValue(
        Command(["xacro ", xacro_path, *xacro_command_args]),
        value_type=str,
    )

    return robot_description_content, launch_args


def generate_robot_description_semantic_content():
    """
    Generates the robot_description_semantic_content, which contains the SRDF (semantic robot description).
    This is typically used for defining robot groups, end-effectors, and kinematics configurations.

    Returns:
        - robot_description_semantic_content: The semantic robot description loaded from the SRDF file.
    """
    robot_description_semantic_path = os.path.join(
    # Uncomment and/or adjust as needed
    #   get_package_share_directory("${robot_name}_moveit_config"),
    #   "config",
    #   "${robot_name}.srdf",
    )

    # Load the SRDF content from the file
    try:
        with open(robot_description_semantic_path, "r") as srdf_file:
            robot_description_semantic_content = srdf_file.read()
    except FileNotFoundError:
        raise RuntimeError(f"SRDF file not found: {robot_description_semantic_path}")


    return robot_description_semantic_content

def extract_cca_ros_viz_setup_params():
    """
    Extracts cca-visualization-related parameters for the robot from the specified yaml file

    Returns:
        - cca_ros_viz_setup_params: List of parameters such as joint_states_topic, planning_group, etc. specified in the cca_<robot>_ros_viz_setup.yaml file in the cca_<robot> package
    """

    cca_ros_viz_setup_params = os.path.join(
        get_package_share_directory('cca_${robot_name}'),
        'config',
        'cca_${robot_name}_ros_viz_setup.yaml'
        )

    return cca_ros_viz_setup_params

def generate_launch_description():
    """
    Generates the full launch description to launch the cca_ros_viz node with robot_description, robot_description_semantic parameters, and RViz with a custom configuration.
    """
    launch_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time (for simulation environments)",
        )
    ]

    (
        robot_description_content,
        description_launch_args,
    ) = generate_robot_description_content()
    robot_description_semantic_content = generate_robot_description_semantic_content()
    cca_ros_viz_setup_params = extract_cca_ros_viz_setup_params()

    robot_description = {"robot_description": robot_description_content}
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    # Default Rviz config file
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("cca_ros_viz"), "rviz", "cca_ros_viz.rviz"]
    )

    # Return the launch description, including robot description-related arguments and RViz node
    return launch.LaunchDescription(
        description_launch_args
        + launch_args
        + [
            # Launch robot_state_publisher
            launch_ros.actions.Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description, use_sim_time],
            ),
            # Launch cca_ros_viz_${robot_name} node
            launch_ros.actions.Node(
                package="cca_ros_viz",
                executable="cca_ros_viz_node",
                name="cca_ros_viz_${robot_name}",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    cca_ros_viz_setup_params,
                    use_sim_time,
                ],
            ),
            # Launch RViz with the specified configuration
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],  # Load RViz config file
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    cca_ros_viz_setup_params,
                    use_sim_time,
                ],
            ),
        ]
    )
EOF

# In the above file replace ${robot_name} which was read as literal due to 'EOF' with the value of that variable
sed -i "s/\${robot_name}/$robot_name/g" \
    $package_name/launch/cca_${robot_name}_viz.launch.py

# Create the affordance planner src file
cat << EOF > $package_name/src/cca_${robot_name}_node.cpp
/*************************************/
// Author: Crasun Jans
// Description:
// This node enables users to plan, visualize, and execute robot joint trajectories for specified tasks. The planning
// process utilizes the Closed-chain Affordance model, as described in the paper:
// "A closed-chain approach to generating affordance joint trajectories for robotic manipulators."
//
// Usage Instructions:
// 1. The framework requires only two inputs: planner configuration and task description. See repo README.md Task
// Examples section for task-description examples.
/*************************************/
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <thread>

class CcaSpot : public cca_ros::CcaRos
{
  public:
    explicit CcaSpot(const std::string &node_name, const rclcpp::NodeOptions &node_options)
        : cca_ros::CcaRos(node_name, node_options)
    {
    }

    // Function to run the planner for a given task and/or execute that task on the robot
    bool run(const cca_ros::PlanningRequest &planning_request)
    {
        motion_status_ = planning_request.status;

        return this->plan_visualize_and_execute(planning_request);
    }
    // Function overload to plan multiple tasks at once
    bool run(const cca_ros::PlanningRequests &planning_requests)
    {
        motion_status_ = planning_requests.status;

        return this->plan_visualize_and_execute(planning_requests);
    }

    // Function to block until the robot completes the planned trajectory
    void block_until_trajectory_execution()
    {
        rclcpp::Rate loop_rate(4);
        auto start_time = std::chrono::steady_clock::now();

        while (*motion_status_ != cca_ros::Status::SUCCEEDED)
        {
            if (*motion_status_ == cca_ros::Status::UNKNOWN)
            {
                RCLCPP_ERROR(this->get_logger(), "Motion was interrupted mid-execution.");
                auto current_time = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 60)
                {
                    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for motion to complete.");
                    return;
                }
            }
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Exiting due to ROS signal");
                return;
            }
            loop_rate.sleep();
        }
    }

  private:
    std::shared_ptr<cca_ros::Status> motion_status_;
    bool includes_gripper_goal_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<CcaSpot>("cca_ros", node_options);

    RCLCPP_INFO(node->get_logger(), "CCA Planner is active");

    // Spin the node so joint states can be read
    std::jthread spinner_thread([node]() { rclcpp::spin(node); });

    /// REQUIRED INPUT: Task description. For quick start, the following block provides an example task description to
    /// do a simple linear motion along the z-axis from the current robot configuration. Edit as needed. See this
    /// package's demo folder or repo README.md for various other examples that cover motions including rotation, screw,
    /// cartesian goal, ee orientation jog, etc. It is also possible to plan multiple of these tasks together as a long
    /// joint trajectory.
    ///------------------------------------------------------------------///
    cca_ros::PlanningRequest req;

    // Specify planning type
    req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

    // Affordance info
    req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    req.task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
    req.task_description.affordance_info.location = Eigen::Vector3d::Zero();

    // Goals
    req.task_description.goal.affordance = 0.1; // Set desired goal for the affordance

    ///------------------------------------------------------------------///

    // Run CCA planner and executor
    if (node->run(req))
    {
        RCLCPP_INFO(node->get_logger(), "Successfully called CCA action");
        node->block_until_trajectory_execution(); // Optionally, block until execution
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "CCA action failed");
        rclcpp::shutdown();
    }

    rclcpp::shutdown();
    return 0;
}
EOF

# Create the CMakeLists file
cat << EOF > $package_name/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)

project(cca_${robot_name})

# Set C++ standard to 20
set(CMAKE_CXX_STANDARD 20)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
# ROS packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cca_ros REQUIRED)

# Non-ROS packages
find_package(Eigen3 REQUIRED)
find_package(affordance_util REQUIRED)
find_package(cc_affordance_planner REQUIRED)

# Specify the node executables
add_executable(\${PROJECT_NAME}_node src/\${PROJECT_NAME}_node.cpp)

# Specify ROS dependencies for the target
ament_target_dependencies(\${PROJECT_NAME}_node rclcpp cca_ros)

# Link Eigen libraries against this project library
target_link_libraries(\${PROJECT_NAME}_node affordance_util::affordance_util cc_affordance_planner::cc_affordance_planner Eigen3::Eigen)

install(TARGETS
  \${PROJECT_NAME}_node
  DESTINATION lib/\${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/\${PROJECT_NAME}
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
echo "Launch files created in $package_name/launch"
echo "Cpp file created in $package_name/src"
