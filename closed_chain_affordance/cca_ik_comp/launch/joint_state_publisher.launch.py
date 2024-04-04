import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                parameters=[{"source_list": ["/spot_driver/joint_states"]}],
                remappings=[("joint_states", "/spot_driver/joint_states")],
            )
        ]
    )
