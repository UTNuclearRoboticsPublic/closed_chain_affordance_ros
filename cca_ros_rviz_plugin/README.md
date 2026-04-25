# CCA RViz Plugin

This package provides the source code for the **CCA RViz Plugin**, which enables intuitive, interactive specification and planning of constrained manipulation tasks, making real-world constrained manipulation accessible to non-expert users. Task examples include opening doors, turning valves, manipulating drawers, moving objects out of the way, and many more.

The plugin is based on the **Closed-Chain Affordance (CCA) framework** (see [references](#references) below), which enables efficient planning of common manipulation tasks by capturing the motions that objects afford to the robot (*task affordances*) compactly as a screw — a unified mathematical representation encompassing rotation, translation, and screw motion. 

The plugin exposes a visual interface in which users describe tasks interactively using an arrow defining the axis of task motion (i.e. the screw axis), the motion type (rotation, translation, or screw), and an affordative grasp pose — the grasp pose that enables the intended task motion. By incorporating real-world context, for instance, via point cloud data, users can seamlessly apply the plugin to real-world tasks, as demonstrated below.

## Planning Types

### 1. Approach
Given a screw axis, its motion type, and an affordative grasp pose, computes a joint trajectory that moves the robot to a target grasp pose along the task path.

> *Example: Move to grasp the valve at its 90° position.*

https://github.com/user-attachments/assets/37b8e0b6-4811-4697-87f7-adb890f1a5c5

### 2. Affordance
Given a screw axis and its motion type, and assuming the robot is already grasping the object, computes a joint trajectory that executes the desired constrained motion along the screw axis.

> *Example: Rotate a valve 180°, or pull a drawer open by 40 cm.*

https://github.com/user-attachments/assets/7557b58b-5c65-4966-8dbc-f8f8744902ce

#### Approach and affordance planning demonstration on real robot

https://github.com/user-attachments/assets/bbd4796e-4a79-4b03-8980-ad6ccaf248c8


### 3. EE Orientation Only
Given a screw axis located at the end-effector, computes a joint trajectory that reorients the EE in place by a desired angle.

https://github.com/user-attachments/assets/cd856ea8-20f6-4f7e-a15a-2b85af197175

### 4. Pose Goal
Given a desired end-effector target pose, computes a joint trajectory to reach that pose.

https://github.com/user-attachments/assets/b5abf628-08f3-4f51-a0ac-8df07bd139d6

## References

- Panthi, J., Alambeigi, F., and Pryor, M. "A Closed-Chain Approach to Generating Affordance Joint Trajectories for Robotic Manipulators." *IEEE Transactions on Robotics*, 2025.

- Panthi, J., Alambeigi, F., and Pryor, M. "Unifying Task-Aware Approach, Grasp, and Post-Grasp Manipulation as a Closed-Chain Mechanism." *IEEE International Conference on Robotics & Automation (ICRA) Workshop on Extreme Manipulation* (Accepted), 2026.
