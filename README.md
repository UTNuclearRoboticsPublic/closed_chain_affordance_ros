# ROS2 Interface for the CCA Planner

This repository provides robot-agnostic ROS2 packages that interface the [Closed-Chain Affordance(CCA) planner](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git) with robotic systems. The CCA planner offers an intuitive approach to planning joint trajectories for robot manipulation tasks that can be thought of as linear, rotational, or screw motions. Defining a task is as simple as specifying an axis, location, and pitch (if applicable). Additionally, it provides the capability to control the end-effector's orientation along the task path.

# Requirements
- `C++20`
- `ROS Humble`

## Dependencies
#### Core Dependency:
- **CCA planner** - Install from [here.](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git)
#### ROS Dependencies
- `moveit`: For self-collision checking
- `moveit_visual_tools`: For visualization of joint movement
- `behaviortree_cpp`: To utilize the CCA Behavior Tree action node
  
With ROS sourced, you may install the ROS dependencies with:
 ```bash
sudo apt install ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-visual-tools ros-${ROS_DISTRO}-behaviortree-cpp
```

## Build Instructions

1. Clone the packages into your ROS2 workspace's `src` folder, for example:
   ```bash
   mkdir -p ~/ws_cca_ros/src && cd ~/ws_cca_ros/src
   git clone git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_ros.git
   ```

2. Build and source the workspace:
   ```bash
   cd ~/ws_cca_ros
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```
## Readily-Supported Robots

An additional `cca_<robot>` package containing robot-specific information is required to launch the planner for a particular robot. Packages are currently available for the following robots, with links provided below. Creating a package for a new robot is simple, quick, and largely automated, as discussed in the [Implementing the Framework on a New Robot](#implementing-the-framework-on-a-new-robot) section.
- [Boston Dynamics Spot robot](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_spot.git)
- [Kinova Gen3 7DoF arm](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_kinova_gen3_7dof.git)

## Rviz CCA Planning Plugin

A user-friendly Rviz plugin is also available and enables visual trajectory planning and execution by simply dragging interactive markers and specifying task types and goals. Launch instructions are provided in the [Interactive Rviz Plugin Planning](#interactive-rviz-plugin-planning) section.

## Implementing the Framework on a New Robot

### Creating the `cca_<robot>` Package

1. Use the package creator script:
   ```bash
   cd ~/ws_cca_ros/src/closed_chain_affordance_ros
   ./cca_robot_package_creator.sh
   ```

2. Configure the generated package:
   - Complete the **`config/cca_<robot>_description.yaml`** file, which contains information about the robot kinematic chain, planning group, relevant action servers, etc. 
   - Complete the **`launch/cca_<robot>_settings.py`** module by providing information about how to generate robot description and robot description semantic parameters.
     Inline comments provide detailed guidance on filling out these files.
   
   - For programmatic trajectory planning and execution, implement task (affordance) details in `cca_<robot>_node.cpp`. Alternatively, you may use the [Rviz plugin](#interactive-rviz-plugin-planning) for interactive planning.

3. Build the new package:
   ```bash
   cd ~/<ros_workspace_name>
   colcon build --packages-select cca_<robot> --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

### Running the Planner

#### Prerequisites

- For real-robot execution, ensure a `follow_joint_trajectory` action server is active, `joint_states` are being published, and TF data is available.  
To plan without a physical robot, simply provide `joint_states` and TF data.

#### Programmatic Trajectory Planning

1. Launch trajectory visualization server:
   ```bash
   ros2 launch cca_<robot> cca_<robot>_val_and_viz.launch.py
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
   ros2 launch cca_<robot> cca_<robot>_val_and_viz.launch.py
   ```

## Author

Janak Panthi (aka Crasun Jans)
