# Affordance Planning Framework - ROS Interface

## Build Instructions
This repository encompasses ROS packages that are used to interface the closed-chain affordance planner with a physical robot. Follow these steps:

1. Clone the cpp branch from this repository and install the planner following the instructions in that branch.

2. Build ROS packages from this branch:
	```bash
	colcon build --packages-select cc_affordance_planner_ros
	```
<small>	This builds the dependencies, `affordance_util_ros` and `moveit_plan_and_viz` as well.</small>

4. To use the planner with the Spot arm, build the following package:
	`cc_affordance_spot_description`

5. To use apriltags to detect affordance, use the following package:
	`apriltag_setup`

## Run Instructions
1. On Spot core, run the driver:
	```bash
	ros2 launch spot_bringup bringup.launch.py hostname:=192.168.50.3
	```

2. On Spot core, run robot state publisher:
	```bash
	ros2 launch spot_description state_publisher.launch.py has_arm:=True
	```

3. On your local machine, launch Moveit motion planning plugin. This is mainly just to publish robot_description_semantic and have Rviz open. It is also convenient to have the motion planning plugin going so as to graphically control robot movements if desired.
	```bash
	ros2 launch spot_arm_moveit2_config spot_arm_planning_execution.launch.py joint_state_topic:=/spot_driver/joint_states
	```

4. On your local machine, run planning visualization server. After that, in Rviz, add MarkerArray with publishing topic, `/ee_trajectory`.
	```bash
	ros2 run moveit_plan_and_viz moveit_plan_and_viz_node
	```

5. On your local machine, run the cc affordance planner node:
	```bash
	ros2 run cc_affordance_planner_ros cc_affordance_planner_ros_node
	```
