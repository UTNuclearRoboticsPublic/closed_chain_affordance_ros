# Author: Crasun Jans

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
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Load cc_affordance_<robot>_ros_setup.yaml onto the ROS parameter server
    cc_affordance_robot_ros_setup = os.path.join(
    get_package_share_directory('cc_affordance_spot_description'),
    'config',
    'cc_affordance_spot_ros_setup.yaml'
    )

    cc_affordance_spot_description_node = Node(
    package='cc_affordance_spot_description',
    executable='cc_affordance_spot_description_node',
    name='cc_affordance_planner_ros_node',
    parameters=[cc_affordance_robot_ros_setup]
    )
    ld.add_action(cc_affordance_spot_description_node)

    # Load robot description onto the ROS parameter server
    # It is not needed to launch the robot description the following way, but the MoveIt motion planning
    # plugin launcher for Spot also loads robot description, and it is convenient to have the
    # motion planning plugin up and ready in Rviz for GUI-based control. So, we run the following launch file:
    # moveit_motion_planning_plugin_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('spot_arm_moveit2_config'), 'launch', 'spot_arm_planning_execution.launch.py')]),
    #     launch_arguments={"joint_state_topic": "/spot_driver/joint_states"}.items()
    # )
    # ld.add_action(moveit_motion_planning_plugin_launch_file)

    # Run robot state publisher
    # For spot, we run it on the Core computer
    # robot_state_publisher_launch_file = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('spot_description'), 'launch', 'state_publisher.launch.py')]),
    #     launch_arguments={"has_arm": "true"}.items()
    # )
    # ld.add_action(robot_state_publisher_launch_file)

    return ld
