# Affordance Planning Framework - ROS Interface

This repository encompasses ROS packages that are used to interface the closed-chain affordance planner with a robot. Follow these steps:

1. Clone the Closed-chain Affordance Cpp libraries from the following repository: [Link to Cpp libraries](https://github.com/UTNuclearRoboticsPublic/closed_chain_affordance.git)

2. Build ROS packages from this branch in the following order:
   - `affordance_util_ros`
   - `moveit_plan_and_viz`

3. After the above two packages are installed, the following packages can be installed in any order (or together):
   - `cc_affordance_planner_ros`
   - `joint_traj_and_tf_recorder`

4. To use the planner with the Boston Dynamics Spot arm, build the following package as well:
   - `cc_affordance_spot_description`

5. To use apriltags to detect affordance, use the following package:
   - `apriltag_setup`

## Maintenance Note
The main branch is maintained and recommended as we almost exclusively use ROS2 now. However, if you must use this branch (ROS1 noetic) and have any problems, please feel free to create an issue, and we'll be happy to help.
