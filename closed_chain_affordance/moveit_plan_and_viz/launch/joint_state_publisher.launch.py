import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="moveit_plan_and_viz",
                executable="moveit_plan_and_viz_node",
                name="moveit_plan_and_viz_server_node",
                output="screen",
                # remappings=[("joint_states", "/spot_driver/joint_states")],
            )
        ]
    )
