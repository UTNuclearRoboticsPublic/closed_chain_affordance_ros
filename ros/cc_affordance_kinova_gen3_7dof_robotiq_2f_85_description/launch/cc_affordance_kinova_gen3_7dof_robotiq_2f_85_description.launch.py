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
    cc_affordance_robot_ros_setup = os.path.join(
    get_package_share_directory('cc_affordance_spot_description'),
    'config',
    'cc_affordance_spot_ros_setup.yaml'
    )

    global_param_node = Node(
    package='cc_affordance_spot_description',
    executable='global_parameter_server',
    name='global_parameter_server',
    parameters=[cc_affordance_robot_ros_setup]
    )
    ld.add_action(global_param_node)

    # Load robot description onto the ROS parameter server here
    #---------------------------------------------------------------#

    # Run robot state publisher here
    #---------------------------------------------------------------#

    return ld
