import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def include_kinova_gen3_launch():
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('kortex_bringup'), '/launch', '/gen3.launch.py'
        ]),
        launch_arguments={
            'robot_ip': 'yyy.yyy.yyy.yyy',
            'use_fake_hardware': 'true',
            'launch_rviz': 'false'
        }.items()
    )


def generate_robot_description_content():
    """
    Generates the robot_description_content, which contains the robot's URDF.
    """
    launch_args = [
        DeclareLaunchArgument(
            'has_arm',
            description='Boolean. Include the Spot Arm.',
            choices=['True', 'False'],
            default_value='True'
        ),
        DeclareLaunchArgument(
            'has_eap',
            description='Boolean. Include the Enhanced Autonomy package (EAP)',
            choices=['True', 'False'],
            default_value='False'
        ),
        DeclareLaunchArgument(
            'has_eap_2',
            description='Boolean. Include the Updated Enhanced Autonomy package (EAP2)',
            choices=['True', 'False'],
            default_value='False'
        ),
        DeclareLaunchArgument(
            'has_realsense',
            description='Boolean. Include an arm-mounted Realsense D435',
            choices=['True', 'False'],
            default_value='False'
        ),
        DeclareLaunchArgument(
            'has_cam_payload',
            description='Boolean. Include the CAM payload',
            choices=['True', 'False'],
            default_value='False'
        ),
    ]

    has_arm = LaunchConfiguration('has_arm')
    has_eap = LaunchConfiguration('has_eap')
    has_eap_2 = LaunchConfiguration('has_eap_2')
    has_realsense = LaunchConfiguration('has_realsense')
    has_cam_payload = LaunchConfiguration('has_cam_payload')

    # Path to the robot xacro file
    this_pkg_share = FindPackageShare('spot_description')
    xacro_path = PathJoinSubstitution([this_pkg_share, 'urdf', 'spot.urdf.xacro'])

    # Generate the robot description by processing the xacro file with the specified configurations
    robot_description_content = ParameterValue(
        Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            ' ',
            xacro_path,
            ' has_arm:=', has_arm,
            ' has_eap:=', has_eap,
            ' has_eap_2:=', has_eap_2,
            ' has_realsense:=', has_realsense,
            ' has_cam_payload:=', has_cam_payload
        ]),
        value_type=str
    )

    return robot_description_content, launch_args

def generate_launch_description():
    launch_args = [DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated time (for simulation environments)'
    )]

    robot_description_content, description_launch_args = generate_robot_description_content()

    # robot_description = {"robot_description": robot_description_content}
    # use_sim_time = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('cca_interactive_goals'),
        'rviz',
        'cca_interactive_goals.rviz'
    ])
        
    return launch.LaunchDescription(
        description_launch_args + launch_args + [
            # Launch cca_ros_viz node
            launch_ros.actions.Node(
                package='cca_interactive_goals',
                executable='cca_interactive_goals_node',
                name='cca_interactive_goals',
                output='screen',
                parameters=[
                    # robot_description,
                    # use_sim_time
                ],
            ),
            # Launch RViz with the specified configuration
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config_file],  # Load RViz config file
                parameters=[
                    # robot_description,
                    # use_sim_time
                ],
            ),
            # Launch Kinova Arm
            include_kinova_gen3_launch(),
        ]
    )
