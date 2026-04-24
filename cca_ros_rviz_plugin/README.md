# CCA RViz Plugin

This package provides the source code for the **CCA RViz Plugin**, which enables intuitive, interactive specification and planning of constrained manipulation tasks — lowering the barrier for non-expert users to work with constrained motion planning.

The plugin is based on the **Closed-Chain Affordance (CCA) framework** (see [references](#references) below), which enables efficient planning of common constrained manipulation tasks by capturing the motions that objects afford to the robot (*task affordances*) compactly as a screw — a unified mathematical representation encompassing rotation, translation, and screw motion. Task examples include opening doors, turning valves, manipulating drawers, moving objects out of the way, and many more.

The plugin exposes a visual interface in which users describe tasks interactively using an arrow defining the axis of task motion (i.e. the screw axis), the motion type (rotation, translation, or screw), and an **affordative grasp pose** — the grasp configuration that enables the intended task motion.

## Planning Types

### 1. Approach
Given a screw axis, its motion type, and an affordative grasp pose, computes a joint trajectory that moves the robot to a target grasp pose along the task path.

> *Example: Move to grasp the valve at its 90° position.*

### 2. Affordance
Given a screw axis and its motion type, and assuming the robot is already grasping the object, computes a joint trajectory that executes the desired constrained motion along the screw axis.

> *Example: Rotate a valve 90°, or pull a drawer open by 40 cm.*

### 3. EE Orientation
Given a screw axis located at the end-effector, computes a joint trajectory that reorients the EE in place by a desired angle.

### 4. Pose Goal
Given a desired end-effector target pose, computes a joint trajectory to reach that pose.

## References

- Panthi, J., Alambeigi, F., and Pryor, M. "A Closed-Chain Approach to Generating Affordance Joint Trajectories for Robotic Manipulators." *IEEE Transactions on Robotics*, 2025.

- Panthi, J., Alambeigi, F., and Pryor, M. "Unifying Task-Aware Approach, Grasp, and Post-Grasp Manipulation as a Closed-Chain Mechanism." *IEEE International Conference on Robotics & Automation (ICRA) Workshop on Extreme Manipulation* (Accepted), 2026.
