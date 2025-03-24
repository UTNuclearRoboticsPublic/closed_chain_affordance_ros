# Closed-Chain Affordance Planning Framework - ROS2 Interface

This repository provides robot-agnostic ROS2 packages that interface the [Closed-Chain Affordance planner](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git) with robotic systems.

## Core Dependencies

Essential dependencies include:
- `affordance_util` and `cc_affordance_planner` from the [Closed-Chain Affordance repository](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git)

### Additional Dependencies

- `moveit`: Motion planning framework
- `moveit_visual_tools`: Visualization utilities for MoveIt
- `behaviortree_cpp`: Behavior tree implementation

## Build Instructions

1. Clone the packages into your ROS2 workspace's `src` folder:
   ```bash
   cd ~/<ros_workspace_name>/src
   git clone git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_ros.git
   ```

2. Build and source the workspace:
   ```bash
   cd ~/<ros_workspace_name>
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Supported Robots

The planner supports the following robots through dedicated packages:
- [Boston Dynamics Spot robot](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_spot.git)
- [Kinova Gen3 7DoF arm](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_kinova_gen3_7dof.git)

## Rviz CCA Planning Plugin

A user-friendly Rviz plugin enables visual trajectory planning and execution by:
- Dragging interactive markers
- Specifying task goals
- Providing intuitive trajectory visualization and manipulation

## Implementing the Framework on a New Robot

### Creating the `cca_<robot>` Package

1. Use the package creator script:
   ```bash
   cd ~/<ros_workspace_name>/src/closed_chain_affordance_ros
   ./cca_robot_package_creator.sh
   ```

2. Configure the generated package:
   - Complete `cca_<robot>_description.yaml` and `cca_<robot>_ros_setup.yaml` in the `config` folder
   - For programmatic trajectory planning and execution, implement task (affordance) details in `cca_<robot>_node.cpp`

3. Build the new package:
   ```bash
   cd ~/<ros_workspace_name>
   colcon build --packages-select cca_<robot>
   source install/setup.bash
   ```

### Running the Planner

#### Prerequisites

- For trajectory execution, ensure a `follow_joint_trajectory` action server is running on the robot

#### Programmatic Trajectory Planning

1. Launch trajectory visualization server:
   ```bash
   ros2 launch cca_<robot> cca_<robot>_viz.launch.py
   ```

2. Run the planner for defined tasks:
   ```bash
   ros2 launch cca_<robot> cca_<robot>.launch.py
   ```

#### Interactive Rviz Plugin Planning

1. Start the CCA ROS action server:
   ```bash
   ros2 launch cca_<robot> cca_<robot>_action_server.launch.py
   ```

2. Launch Rviz with the interactive planning plugin:
   ```bash
   ros2 launch cca_<robot> cca_<robot>_viz.launch.py
   ```

## Author

Janak Panthi (aka Crasun Jans)
