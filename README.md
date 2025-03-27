# ROS2 Interface for the CCA Planner

This repository provides robot-agnostic ROS2 packages that interface the [Closed-Chain Affordance(CCA) planner](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git) with robotic systems. The CCA planner offers an intuitive approach to planning joint trajectories for robot manipulation tasks that can be thought of as linear, rotational, or screw motions. Defining a task is as simple as specifying an axis, location, and pitch (if applicable). Additionally, it provides the capability to control the end-effector's orientation along the task path.

## Core Dependencies

- `affordance_util` and `cc_affordance_planner` packages from the [Closed-Chain Affordance repository](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git)

### Optional Dependencies
- `moveit`: For self-collision checking
- `moveit_visual_tools`: For visualization of joint movement
- `behaviortree_cpp`: To utilize the CCA Behavior Tree action node

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
## Readily-Supported Robots

An additional `cca_<robot>` package containing robot-specific information is required to launch the planner for a particular robot. Packages are currently available for the following robots, with links provided below. Creating a package for a new robot is straightforward and largely automated, as discussed in the [Implementing the Framework on a New Robot](#implementing-the-framework-on-a-new-robot) section.
- [Boston Dynamics Spot robot](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_spot.git)
- [Kinova Gen3 7DoF arm](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_kinova_gen3_7dof.git)

## Rviz CCA Planning Plugin

A user-friendly Rviz plugin is also available and enables visual trajectory planning and execution by simply dragging interactive markers and specifying task types and goals. Launch instructions are provided in the [Interactive Rviz Plugin Planning](#interactive-rviz-plugin-planning) section.

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
