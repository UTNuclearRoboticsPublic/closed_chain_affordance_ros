# Closed-Chain Affordance Planning Framework - ROS2 Interface
This repository contains robot-agnostic ROS2 packages used to interface the Closed-Chain Affordance planner with a robot.

## Ongoing work and remaining tasks

The work that I (John Lyle) have done on this project consists of three primary packages detailed below. The primary purpose of this project is to create a UI tool and planning request builder for the CCA package. This is being done using a custom RVIZ2 panel that creates a series of buttons that the user can select options through. It also creates interactive markers that the user can move around to specify a motion or frame.

1. `cca_interactive_goals` This package contains all of the code for the custom RVIZ2 panel and the corresponding node. This is a rather lengthy file as it contains the code for creating the UI however it is mostly finished and very few areas need to be interacted with.

2. `plan_req_builder` This package contains the node for the creation of the planning request and task description based on the information received from the UI. This is a separate package since the node contained within the custom RVIZ2 panel cannot be launched using parameters which is necessary for the node creating the planning request since there are certain things that are robot dependent.

3. `interactive_goal_interfaces` This is a simple custom ros2 interfaces package. It has a few message types that are used for communicating planning request information between the UI panel and the planning request builder.

### Interactive goals dependencies

Outside of the above 3 packages there are no dependencies not included in the standard ros2 installation except for CCA and related libraries.
Ensure that cca_ros and cca_spot are on the correct branches.

### Installation Instructions

The interfaces package needs to be built prior to the other two packages since it is a dependency.

```
colcon build --packages-select intercative_goal_interfaces cca_interactive_goals plan_req_builder
```

### interactive_goal_interfaces

The code for this package is pretty self explanatory and follows the structure outlined in the ROS2 tutorials for a custom interfaces package. Each of the message file has comments detailing each message and it's usage. This package is completed and should not need to be touched however if the structure of the messages needs to change it should be rather easy to change.

### cca_interactive_goals

This package follows the structure of a custom RVIZ2 panel as detailed in the ROS2 tutorials. It is also a node which adds a bit of complexity but the base structure of the package has already been setup.

Important Files

1. `panel.hpp` Header file, should not need to be changed very much.
2. `panel.cpp` Main file where the panel is created. This primarily consists of a function that sets up the UI and links buttons to callback functions. The rest of the file is the callback functions for each button and have all been commented with the necessary information.

TODO: The following items still need to be completed/modified for this aspect of the project. 

1. Replace hardcoded tf names. In the creation of the interactive marker objects the tf `base_link` is used. This will change depending on the robot and needs to be received post panel initialization. I am not sure if the interactive marker header frame_id can be changed after it is created but if so this would be any easy solution to receive the tf name from some ros2 topic and change it. If the name cannot be changed then the creation of the interactive markers will need to be postponed till after the tf names are received.
2. Implement arrow rotations for in-place end-effector orientation. When an axis is selected the arrow should orient itself with that axis. I have tried to implement this in the function `axisOptionSelected` however the attempted implementation does not work properly.
3. 

### plan_req_builder

This package contains a node capable of being launched with parameter files. This node receives information from the UI panel and the interactive markers to modify settings and create planning requests.

Important Files

1. `plan_req_builder.hpp` Header file, should not need to be changed very much.
2. `plan_req_builder.cpp` Main file for the node, primarily used to create planning requests from messages sent by the panel.

TODO: The following items still need to be completed/modified for this aspect of the project.

1. Debugging print outs. Right now the planning requests are being built and sent to CCA however they are not being visualized/the motion generated is not correct. CCA is also not throwing an error for bad planning requests so print outs detailing the planning requests would help with this.
2. Planner config setting changes need to be implemented from the message containing settings to the actual planner config.

### Additional remaining tasks

1. Testing. Most of what needs to be done requires testing the functionality of the various planning requests.
2. In place end effector orientation control. The correct response from the UI and interactive marker for selecting the orientation still needs to be finished.
3. Finish the implementation of the Stop button. Currently the stop button does not work and the code has to be relaunched everytime a new planning request needs to be created. This will likely mean messing with the function `block_until_trajectory_execution`

### Instructions for running current code and testing

1. Build all of the necessary packages.
2. Open 3 terminals and source the ros2 install files
3. In the three terminals enter the following commands, one in each terminal.
`ros2 launch spot_description state_publisher.launch.py has_arm:=True `
`ros2 launch cca_spot cca_spot_interactive_goals.launch.py`
`ros2 launch cca_spot cca_spot_viz.launch.py`
4. From the Panels -> Add New Panel menu select CcaInteractiveGoals
5. Add an interactive marker display and ensure it is set to `interactive_goals`
6. The interactive goal panel is now ready to be used.

### Final Recommendations for Future Work

1. Understand the goal of the project and the 4 types of requests. Clicking through the panel and following the workflow it presents is a good tool for doing this.
2. Read through the comments of the panel code and focus on the callback functions for understanding the functionality.

### Notable Dependencies
1. The `cca_ros_viz` package provides a ROS service to visualize the planned trajectory in Rviz. To utilize this functionality, ensure moveit is installed along with the `moveit_visual_tools` package.
2. For AprilTag usage, install the ROS apriltag package with `sudo apt install ros-<ROS_DISTRO>-apriltag`.

## Build Instructions

1. Install the Closed-chain Affordance Cpp libraries by following instructions from this repository:</br>
   [Cpp library installation instructions](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git)

2. Clone the packages from this branch into your ROS2 workspace's `src` folder and build and source them:
   ```
   cd ~/<ros_workspace_name>/src
   ```
   ```
   git clone git@github.com:UTNuclearRoboticsPublic/closed_chain_affordance_ros.git
   ```
   ```
   cd ~/<ros_workspace_name>
   ```
   ```
   colcon build
   ```
   ```
   source install/setup.bash
   ```

## Run Instructions
The entry point for this planner is a launch file in a package named as `cca_<robot>` that contains robot-related information for closed-chain affordance planning. The framework is implemented for the following robots, and the links direct you to the repositories containing those packages and instructions on how to run the planner for each robot.
   - [Boston Dynamics Spot robot](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_spot.git)
   - [Kinova Gen3 7DoF arm](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance_kinova_gen3_7dof.git)

## Instructions to Implement the Framework on a New Robot

### Creating and Building the `cca_<robot>` Package

Implementing this framework on a new robot is straightforward. All you need to do is create and build the `cca_<robot>` package that contains robot-related information for closed-chain affordance planning. To streamline this package creation, a  script is available. Navigate to the `closed_chain_affordance_ros` directory and run the package-creator script.
   ```
   cd ~/<ros_workspace_name>/src/closed_chain_affordance_ros
   ```
   ```
   ./cca_robot_package_creator.sh
   ```

The package will contain two files in its `config` folder: `cca_<robot>_description.yaml` and `cca_<robot>_ros_setup.yaml`, which will have empty fields with instructions to fill out. Fill them out. Next, to execute an affordance with the robot, provide affordance information in the `cca_<robot>_node.cpp` file located in the `src` folder of the package. Once this is done, build and source the package:
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
   - A `follow_joint_trajectory` action server is running on the robot to receive joint trajectory commands.
   - `robot_description` is loaded with the `cca_ros_viz` node.

2. Run the service server to visualize the planned trajectory in Rviz with the following command:
   ```
   ros2 launch cca_<robot> cca_<robot>_viz.launch.py
   ```

3. Run the planner:
   ```
   ros2 launch cca_<robot> cca_<robot>.launch.py
   ```

## Author
Janak Panthi aka Crasun Jans
