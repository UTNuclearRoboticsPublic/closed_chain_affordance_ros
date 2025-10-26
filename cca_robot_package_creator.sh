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
# **Info for the ${robot_name} robot to build description from this yaml** #
# If a URDF is available, use cca_${robot_name}_urdf.yaml instead.
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

# Create the description file
cat << EOF > $package_name/config/cca_${robot_name}_urdf.yaml
# **Info for the ${robot_name} robot to build description from URDF** #
# Provide the reference frame, kinematic chain, end effector and tool info.
ref_frame:
  - name: # Example: base_link

kinematic_chain:
  - base_joint_name: # Example: joint_1
    end_joint_name: # Example: joint_6

end_effector:
  - frame_name: # Example: ee_link
    gripper_joint_name: # Example: joint_6 # Unused but provide one valid joint name

tool: # This does not have to be in the URDF, and is usually located at the center of the palm.
  - name: # Example: robot_tool 
    offset_from_ee_frame: # Example: [0.0, 0.0, 0.02] # Tool location in the EE frame
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
"""ROS 2 launch script to launch the CCA planner for the tasks defined in src/cca_${robot_name}_node.cpp

Author: Crasun Jans

This script launches the CCA planner for the tasks defined in src/cca_${robot_name}_node.cpp by loading the node with
robot-specific CCA settings and robot description. You may also launch the node under gdb for debugging
by using the debug arg.

The robot-specific configuration is automatically imported from cca_<robot>_settings.py in the
same directory.

### Usage:
    ros2 launch cca_${robot_name} cca_${robot_name}.launch.py
    ros2 launch cca_${robot_name} cca_${robot_name}.launch.py debug:=true
====================================================================
THIS FILE IS AUTOGENERATED AND USUALLY DOES NOT REQUIRE MODIFICATION.
====================================================================
"""

import os, importlib.util
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_settings_module(package='cca_${robot_name}', module_name='cca_${robot_name}_settings'):
    """Dynamically load the CCA settings module from a package's launch folder."""
    try:
        settings_path = os.path.join(
            FindPackageShare(package).find(package), 'launch', f'{module_name}.py'
        )
        spec = importlib.util.spec_from_file_location(module_name, settings_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        print(f"[INFO] Loaded settings module: {settings_path}")
        return module
    except Exception as e:
        print(f"[ERROR] Failed to load settings module '{module_name}' from package '{package}': {e}")
        raise RuntimeError(f"Unable to import {module_name} from {package}/launch") from e


# Load module and expose key functions
cca_robot_settings = load_settings_module()
declare_launch_args = cca_robot_settings.declare_launch_args
define_robot_paths_and_settings = cca_robot_settings.define_robot_paths_and_settings
generate_robot_description_content = cca_robot_settings.generate_robot_description_content

def generate_launch_description():
    """Generate the complete launch description for the CCA action server.

    Returns:
        LaunchDescription: Complete launch configuration
    """

    # Get robot-specific configuration
    args = declare_launch_args()
    robot_args = args["robot_args"]
    base_args = args["base_args"]
    settings = define_robot_paths_and_settings()

    # Generate robot description
    robot_description = generate_robot_description_content(
        package_name=settings["urdf_package"],
        urdf_rel_path=settings["urdf_xacro_path"],
        launch_args=robot_args,
    )

    # Get CCA ROS setup parameters for the robot
    cca_robot_setup_params = settings["cca_robot_ros_setup_path"]

    # Declare debug mode argument
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description="Enable GDB debug mode for the CCA planner node.",
    )

    debug = LaunchConfiguration("debug")
    node_prefix = PythonExpression([
        "'xterm -e gdb -ex run --args' if '", debug, "' == 'true' else ''"
    ])
    emulate_tty = PythonExpression(["'", debug, "' == 'true'"])

    # Node parameters
    params = [
        {"robot_description": robot_description},
        cca_robot_setup_params,
    ]

    return LaunchDescription(
        robot_args
        + base_args
        + [debug_arg]
        + [
            Node(
                package="cca_${robot_name}",
                executable="cca_${robot_name}_node",
                name="cc_affordance_planner_ros",
                output="screen",
                prefix=[node_prefix],
                emulate_tty=emulate_tty,
                parameters=params,
            ),
        ]
    )
EOF

# In the above file replace ${robot_name} which was read as literal due to 'EOF' with the value of that variable
sed -i "s/\${robot_name}/$robot_name/g" \
    $package_name/launch/cca_${robot_name}.launch.py

# Create the action server launch file
cat << 'EOF' > $package_name/launch/cca_${robot_name}_action_server.launch.py
"""ROS 2 launch script to launch the CCA planner as an action server.

Author: Crasun Jans

This script launches the CCA planner as a ROS action server by loading `cca_ros_action_node` with
robot-specific CCA settings and robot description. You may also load the node under gdb for debugging
by using the debug arg.

The robot-specific configuration is automatically imported from cca_<robot>_settings.py in the
same directory.

### Usage:
    ros2 launch cca_${robot_name} cca_${robot_name}_action_server.launch.py
    ros2 launch cca_${robot_name} cca_${robot_name}_action_server.launch.py debug:=true
====================================================================
THIS FILE IS AUTOGENERATED AND USUALLY DOES NOT REQUIRE MODIFICATION.
====================================================================
"""

import os, importlib.util
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_settings_module(package='cca_${robot_name}', module_name='cca_${robot_name}_settings'):
    """Dynamically load the CCA settings module from a package's launch folder."""
    try:
        settings_path = os.path.join(
            FindPackageShare(package).find(package), 'launch', f'{module_name}.py'
        )
        spec = importlib.util.spec_from_file_location(module_name, settings_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        print(f"[INFO] Loaded settings module: {settings_path}")
        return module
    except Exception as e:
        print(f"[ERROR] Failed to load settings module '{module_name}' from package '{package}': {e}")
        raise RuntimeError(f"Unable to import {module_name} from {package}/launch") from e


# Load module and expose key functions
cca_robot_settings = load_settings_module()
declare_launch_args = cca_robot_settings.declare_launch_args
define_robot_paths_and_settings = cca_robot_settings.define_robot_paths_and_settings
generate_robot_description_content = cca_robot_settings.generate_robot_description_content

def generate_launch_description():
    """Generate the complete launch description for the CCA action server.

    Returns:
        LaunchDescription: Complete launch configuration
    """

    # Get robot-specific configuration
    args = declare_launch_args()
    robot_args = args["robot_args"]
    base_args = args["base_args"]
    settings = define_robot_paths_and_settings()

    # Generate robot description
    robot_description = generate_robot_description_content(
        package_name=settings["urdf_package"],
        urdf_rel_path=settings["urdf_xacro_path"],
        launch_args=robot_args,
    )

    # Get CCA ROS setup parameters for the robot
    cca_robot_setup_params = settings["cca_robot_ros_setup_path"]

    # Declare debug mode argument
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description="Enable GDB debug mode for the CCA planner node.",
    )

    debug = LaunchConfiguration("debug")
    node_prefix = PythonExpression([
        "'xterm -e gdb -ex run --args' if '", debug, "' == 'true' else ''"
    ])
    emulate_tty = PythonExpression(["'", debug, "' == 'true'"])

    # Node parameters
    params = [
        {"robot_description": robot_description},
        cca_robot_setup_params,
    ]

    return LaunchDescription(
        robot_args
        + base_args
        + [debug_arg]
        + [
            Node(
                package="cca_ros_action",
                executable="cca_ros_action_node",
                name="cc_affordance_planner_ros",
                output="screen",
                prefix=[node_prefix],
                emulate_tty=emulate_tty,
                parameters=params,
            ),
        ]
    )
EOF

# In the above file replace ${robot_name} which was read as literal due to 'EOF' with the value of that variable
sed -i "s/\${robot_name}/$robot_name/g" \
    $package_name/launch/cca_${robot_name}_action_server.launch.py

# Create the visualization server/RVIZ plugin launch file
cat << 'EOF' > $package_name/launch/cca_${robot_name}_viz.launch.py
"""ROS 2 launch script for CCA robot trajectory validation and optional visualization with RViz.

Author: Crasun Jans

This launch script visualizes and validates joint trajectories generated by the
Closed-Chain Affordance (CCA) planner. It launches:
- robot_state_publisher: Publishes robot transforms from URDF
- cca_ros_viz_node: Validates and visualizes CCA trajectories
- rviz2: Interactive 3D visualization (optional)

The robot-specific configuration is automatically imported from cca_<robot>_settings.py in the
same directory.

### Usage:
    ros2 launch cca_${robot_name} cca_${robot_name}_ros_viz.launch.py
    ros2 launch cca_${robot_name} cca_${robot_name}_ros_viz.launch.py launch_rviz:=true
====================================================================
THIS FILE IS AUTOGENERATED AND USUALLY DOES NOT REQUIRE MODIFICATION.
====================================================================
"""

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os, importlib.util
from launch_ros.substitutions import FindPackageShare

def load_settings_module(package='cca_${robot_name}', module_name='cca_${robot_name}_settings'):
    """Dynamically load the CCA settings module from a package's launch folder."""
    try:
        settings_path = os.path.join(
            FindPackageShare(package).find(package), 'launch', f'{module_name}.py'
        )
        spec = importlib.util.spec_from_file_location(module_name, settings_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    except Exception as e:
        print(f"[ERROR] Failed to load settings module '{module_name}' from package '{package}': {e}")
        raise RuntimeError(f"Unable to import {module_name} from {package}/launch") from e

# Load module and expose its functions
cca_robot_settings = load_settings_module()
declare_launch_args = cca_robot_settings.declare_launch_args
define_robot_paths_and_settings = cca_robot_settings.define_robot_paths_and_settings
generate_robot_description_content = cca_robot_settings.generate_robot_description_content
generate_robot_description_semantic_content = cca_robot_settings.generate_robot_description_semantic_content


def generate_launch_description():
    """Generate the complete launch description for robot visualization.

    Returns:
        LaunchDescription: Complete launch configuration
    """
    # Get robot-specific configuration
    args = declare_launch_args()
    robot_args = args["robot_args"]
    base_args = args["base_args"]
    settings = define_robot_paths_and_settings()

    # Generate robot descriptions
    robot_description = generate_robot_description_content(
        package_name=settings["urdf_package"],
        urdf_rel_path=settings["urdf_xacro_path"],
        launch_args=robot_args,
    )

    robot_description_semantic = generate_robot_description_semantic_content(
        package_name=settings["srdf_package"],
        srdf_path=settings["srdf_path"],
        all_launch_args=robot_args,
        srdf_arg_usage=settings["srdf_arg_usage"],
        srdf_subset_args=settings["srdf_subset_args"],
    )

    # Get CCA validation and visualization setup parameters
    cca_viz_params = settings["cca_robot_ros_viz_setup_path"]

    # RViz configuration
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cca_ros_viz"), "rviz", "cca_ros_viz.rviz"]
    )

    # Prepare parameters
    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}
    params_common = [
        {"robot_description": robot_description},
        {"robot_description_semantic": robot_description_semantic},
        cca_viz_params,
        use_sim_time,
    ]

    # Define nodes
    return LaunchDescription(
        robot_args
        + base_args
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=params_common,
            ),
            Node(
                package="cca_ros_viz",
                executable="cca_ros_viz_node",
                name="cca_ros_viz",
                output="screen",
                parameters=params_common,
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                arguments=["-d", rviz_config],
                parameters=params_common,
            ),
        ]
    )
EOF

# In the above file replace ${robot_name} which was read as literal due to 'EOF' with the value of that variable
sed -i "s/\${robot_name}/$robot_name/g" \
    $package_name/launch/cca_${robot_name}_viz.launch.py

# Create the python module that houses robot-related settings such as CCA ros-setup, robot description, etc.
cat << 'EOF' > $package_name/launch/cca_${robot_name}_settings.py
"""Robot-specific CCA settings for ${robot_name}.

Author: Crasun Jans

This module centralizes all robot-specific configuration for ${robot_name}
for usage with the Closed-Chain Affordance (CCA) planner. It provides:

1. Launch arguments for robot hardware configuration
2. File paths for URDF, SRDF, and CCA configuration files
3. Functions to generate robot_description and robot_description_semantic content

### To Customize for Your Robot:
Replace the robot-specific configuration in:
- declare_launch_args(): Define your robot's hardware/configuration options
- define_robot_paths_and_settings(): Specify your robot's package names and file paths

The generic functions at the bottom should work for any robot and typically don't
need modification.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# ---------------------------------------------------------------------------
# 1️⃣ USER-EDITABLE: Declare Launch Arguments
# ---------------------------------------------------------------------------
def declare_launch_args():
    """Declare all launch arguments for robot configuration.

    ✅ Modify the ROBOT ARGUMENTS section for your robot's hardware options.
    ⚠️ The BASE ARGUMENTS section should NOT be modified (used by launch files).

    Returns:
        dict: Dictionary with 'robot_args' and 'base_args' keys containing
              lists of DeclareLaunchArgument objects
    """
    # ---- ROBOT-SPECIFIC ARGS (Modify as needed) ----
    robot_args = [
        # Example:
        # DeclareLaunchArgument(
        #     "has_arm",
        #     default_value="True",
        #     choices=["True", "False"],
        #     description="Include the Spot arm.",
        # ),
    ]

    # ---- BASE / SYSTEM ARGS (DO NOT MODIFY) ----
    # These are standard arguments used by launch files
    base_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            choices=["true", "false"],
            description="Use simulation time (true for Gazebo or rosbag playback).",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            choices=["true", "false"],
            description="Whether to launch RViz for visualization.",
        ),
    ]

    return {"robot_args": robot_args, "base_args": base_args}


# ---------------------------------------------------------------------------
# 2️⃣ USER-EDITABLE: Define Robot Paths and Settings
# ---------------------------------------------------------------------------
def define_robot_paths_and_settings():
    """Define robot-specific package names, file paths, and configuration settings.

    ⚠️ Modify package names and file paths for your robot.
       Keep the key names unchanged so launch files can access them consistently.

    Returns:
        dict: Configuration dictionary with the following keys:
            - urdf_package: Package containing robot URDF
            - urdf_xacro_path: Relative path to URDF xacro file
            - srdf_package: Package containing robot SRDF
            - srdf_path: Relative path to SRDF file
            - srdf_arg_usage: How SRDF uses launch args ("all", "none", or "subset")
            - srdf_subset_args: List of arg names if using "subset" mode
            - cca_robot_package: Package containing CCA configuration files
            - cca_robot_ros_setup_path: Path to ROS setup YAML
            - cca_robot_ros_viz_setup_path: Path to visualization setup YAML
            - cca_robot_description_path: Path to CCA robot description YAML
            - cca_robot_urdf_path: Path to CCA URDF configuration YAML
    """
    return {
        # ---- ROBOT DESCRIPTION - URDF (Modify as needed) ----
        "urdf_package": "", # Example: spot_description
        "urdf_xacro_path": "", # Example: urdf/spot.urdf.xacro
        
        # ---- ROBOT DESCRIPTION SEMANTIC  - SRDF (Modify as needed) ----
        "srdf_package": "", # Example spot_moveit_config
        "srdf_path": "", # Example: config/spot.srdf.xacro
        
        # SRDF argument reuse options:
        # - "all": Pass all robot_args to SRDF xacro
        # - "none": Don't pass any arguments to SRDF
        # - "subset": Pass only the arguments listed in srdf_subset_args
        "srdf_arg_usage": "", # Example: "all"
        "srdf_subset_args": [],  # Used if srdf_arg_usage == "subset", Example: "has_arm", "kinematic_model"
        
        # ---- CCA CONFIGURATION FILES (AUTOGENERATED) ----
        "cca_robot_package": "cca_${robot_name}",
        "cca_robot_ros_setup_path": os.path.join(
            get_package_share_directory("cca_${robot_name}"),
            "config",
            "cca_${robot_name}_ros_setup.yaml"
        ),
        "cca_robot_ros_viz_setup_path": os.path.join(
            get_package_share_directory("cca_${robot_name}"),
            "config",
            "cca_${robot_name}_ros_viz_setup.yaml"
        ),
        "cca_robot_description_path": os.path.join(
            get_package_share_directory("cca_${robot_name}"),
            "config",
            "cca_${robot_name}_description.yaml"
        ),
        "cca_robot_urdf_path": os.path.join(
            get_package_share_directory("cca_${robot_name}"),
            "config",
            "cca_${robot_name}_urdf.yaml"
        ),
    }


# ---------------------------------------------------------------------------
# GENERIC FUNCTIONS (Should work for any robot - rarely needs modification)
# ---------------------------------------------------------------------------
def generate_robot_description_content(
    package_name: str, urdf_rel_path: str, launch_args: list
):
    """Generate the robot_description parameter content.

    Automatically detects whether the file is xacro or static URDF and processes
    it accordingly. For xacro files, all launch arguments are passed through.

    Args:
        package_name: ROS 2 package containing the URDF
        urdf_rel_path: Relative path to URDF file within the package
        launch_args: List of DeclareLaunchArgument objects to pass to xacro

    Returns:
        ParameterValue or str: Robot description content
            - ParameterValue: If file is xacro (will be evaluated at launch time)
            - str: If file is static URDF (loaded immediately)

    Raises:
        RuntimeError: If URDF file is not found

    Example:
        >>> robot_desc = generate_robot_description_content(
        ...     "my_robot_description",
        ...     "urdf/robot.urdf.xacro",
        ...     robot_args
        ... )
    """
    urdf_full_path = os.path.join(
        get_package_share_directory(package_name), urdf_rel_path
    )

    # Handle xacro files - process with launch arguments
    if urdf_full_path.endswith(".xacro"):
        xacro_args = []
        for arg in launch_args:
            xacro_args.extend([f" {arg.name}:=", LaunchConfiguration(arg.name)])

        urdf_file = PathJoinSubstitution(
            [FindPackageShare(package_name), urdf_rel_path]
        )
        return ParameterValue(
            Command(["xacro ", urdf_file, *xacro_args]), value_type=str
        )

    # Handle static URDF files - load directly
    try:
        with open(urdf_full_path, "r", encoding="utf-8") as f:
            return f.read()
    except FileNotFoundError as e:
        raise RuntimeError(
            f"URDF file not found: {urdf_full_path}\n"
            f"Ensure '{package_name}' package is installed and contains '{urdf_rel_path}'"
        ) from e


def generate_robot_description_semantic_content(
    package_name: str,
    srdf_path: str,
    all_launch_args: list,
    srdf_arg_usage: str,
    srdf_subset_args: list,
):
    """Generate the robot_description_semantic parameter content.

    Supports both static and xacro-based SRDF files with flexible argument passing.
    The srdf_arg_usage parameter controls which launch arguments are passed to xacro.

    Args:
        package_name: ROS 2 package containing the SRDF
        srdf_path: Relative path to SRDF file within the package
        all_launch_args: List of all available DeclareLaunchArgument objects
        srdf_arg_usage: Argument passing mode:
            - "all": Pass all launch arguments to SRDF xacro
            - "none": Don't pass any arguments (for static SRDF or parameterless xacro)
            - "subset": Pass only arguments listed in srdf_subset_args
        srdf_subset_args: List of argument names to use when srdf_arg_usage == "subset"

    Returns:
        ParameterValue or str: Semantic robot description content
            - ParameterValue: If file is xacro (will be evaluated at launch time)
            - str: If file is static SRDF (loaded immediately)

    Raises:
        RuntimeError: If SRDF file is not found

    Example:
        >>> robot_semantic = generate_robot_description_semantic_content(
        ...     "my_robot_moveit_config",
        ...     "config/robot.srdf.xacro",
        ...     robot_args,
        ...     "subset",
        ...     ["has_gripper", "arm_type"]
        ... )
    """
    srdf_full_path = os.path.join(get_package_share_directory(package_name), srdf_path)

    # Handle xacro files - process with selected arguments
    if srdf_full_path.endswith(".xacro"):
        # Select which arguments to pass based on usage mode
        if srdf_arg_usage == "none":
            selected_args = []
        elif srdf_arg_usage == "subset":
            selected_args = [a for a in all_launch_args if a.name in srdf_subset_args]
        else:  # "all"
            selected_args = all_launch_args

        # Build xacro command with selected arguments
        xacro_args = []
        for arg in selected_args:
            xacro_args.extend([f" {arg.name}:=", LaunchConfiguration(arg.name)])

        srdf_file = PathJoinSubstitution(
            [FindPackageShare(package_name), srdf_path]
        )
        return ParameterValue(
            Command(["xacro ", srdf_file, *xacro_args]), value_type=str
        )

    # Handle static SRDF files - load directly
    try:
        with open(srdf_full_path, "r", encoding="utf-8") as f:
            return f.read()
    except FileNotFoundError as e:
        raise RuntimeError(
            f"SRDF file not found: {srdf_full_path}\n"
            f"Ensure '{package_name}' package is installed and contains '{srdf_path}'"
        ) from e
EOF

# In the above file replace ${robot_name} which was read as literal due to 'EOF' with the value of that variable
sed -i "s/\${robot_name}/$robot_name/g" \
    $package_name/launch/cca_${robot_name}_settings.py

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

class CcaRobot : public cca_ros::CcaRos
{
  public:
    explicit CcaRobot(const std::string &node_name, const rclcpp::NodeOptions &node_options)
        : cca_ros::CcaRos(node_name, node_options)
    {
    }

    // Function to run the planner for a given task and/or execute that task on the robot
    bool run(const cca_ros::PlanningRequest &planning_request)
    {

	cca_ros::PlanningResponse response = this->plan(planning_request);
        motion_status_ = response.status;
	return response.result.success;
    }
    // Function overload to plan multiple tasks at once
    bool run(const std::vector<cca_ros::PlanningRequest> &planning_requests)
    {

	cca_ros::PlanningResponse response = this->plan(planning_requests);
        motion_status_ = response.status;
	return response.result.success;
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<CcaRobot>("cca_ros", node_options);

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
