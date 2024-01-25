# Affordance Planning Framework - ROS Interface

This repository encompasses ROS packages that are used to interface the closed-chain affordance planner with a physical robot. Follow these steps:

1. Clone the cpp branch from this repository and install the planner following the instructions in that branch.

2. Build ROS packages from this branch in the following order:
   - `affordance_util_ros`
   - `moveit_plan_and_viz`

3. After the above two packages are installed, the following packages can be installed in any order (or together):
   - `cc_affordance_planner_ros`
   - `joint_traj_and_tf_recorder`

4. To use the planner with the Spot arm, build the following package as well:
   - `cc_affordance_spot_description`

5. To use apriltags to detect affordance, use the following package:
   - `apriltag_setup`
