# Closed-Chain Affordance Planning Framework - ROS2 Interface

## Build Instructions
This repository contains ROS2 packages used to interface the closed-chain affordance planner with a physical robot. Follow these steps:

1. Install the closed-chain affordance Cpp libraries by following instructions on the `cpp` branch of this repository:</br>
   [Cpp library installation instructions](https://github.com/UTNuclearRobotics/closed_chain_affordance/tree/cpp)

2. Clone the packages from this branch into your ROS2 workspace's `src` folder and build and source them:
   ```
   cd ~/<ros_workspace_name>/src
   ```
   ```
   git clone -b cpp git@github.com:UTNuclearRobotics/closed_chain_affordance.git
   ```
   ```
   cd ~/<ros_workspace_name>
   ```
   ```
   colcon build --packages-select affordance_util_ros moveit_plan_and_viz cc_affordance_planner_ros
   ```
   ```
   source install/setup.bash
   ```

3. If using an AprilTag to detect affordance, also build and source the following package (TODO):
   ```
   colcon build --packages-select apriltag_setup
   ```
   ```
   source install/setup.bash
   ```

## Run Instructions
The entry point for this planner is a launch file in a package named as `cca_<robot>` that contains robot-related information for closed-chain affordance planning. The framework is implemented for the following robots, and the links direct you to the repositories containing those packages and instructions on how to run the planner for each robot.
   - [Boston Dynamics Spot robot](https://github.com/UTNuclearRobotics/closed_chain_affordance_spot.git)
   - [Kinova Gen3 7DoF arm](https://github.com/UTNuclearRobotics/closed_chain_affordance_kinova.git)

## Instructions to Implement the Framework on a New Robot

### Creating and Building the `cca_<robot>` Package

Implementing this framework on a new robot is straightforward. All you need to do is create and build the `cca_<robot>` package that contains robot-related information for closed-chain affordance planning. To streamline this package creation, a  script is available. Navigate to the `closed_chain_affordance` directory of this repository and run the script.
   ```
   cd ~/<ros_workspace_name>/src/closed_chain_affordance
   ```
   ```
   ./cca_robot_package_creator.sh
   ```

The package will contain two files in its `config` folder: `cca_<robot>_description.yaml` and `cca_<robot>_ros_setup.yaml`, which will have empty fields with instructions to fill out. Fill them out. Next, to execute an affordance with the robot, provide affordance information in the `cca_<robot>.cpp` file located in the `src` folder of the package. Once this is done, build and source the package:
   ```
   cd ~/<ros_workspace_name>
   ```
   ```
   colcon build --packages-select cca_<robot>
   ```
   ```
   source install/setup.bash
   ```

### Running the Planner

1. Before running this framework, the following three things need to happen. Since there are numerous ways to accomplish this, depending on your system, we do not provide specific instructions here.
   - A `follow_joint_trajectory` action server is running on the robot to receive commands.
   - `robot_description` has been loaded onto the parameter server.
   - `robot_state_publisher` is running to publish TF data to RViz.

2. Run the service server to visualize the planned trajectory in RViz with the following command:
   ```
   ros2 run moveit_plan_and_viz moveit_plan_and_viz_node
   ```

3. Run the planner, replacing `<robot>` with the robot name used to create the `cca_<robot>` package:
   ```
   ros2 launch cca_<robot> cca_<robot>.launch.py
   ```
