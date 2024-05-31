from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("spot").to_moveit_configs()

    cca_ik_comp_node = Node(
        package="cca_ik_comp",
        executable="cca_ik_comp_node",
        prefix=["xterm -e gdb -ex run --args"],
        emulate_tty=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([cca_ik_comp_node])
