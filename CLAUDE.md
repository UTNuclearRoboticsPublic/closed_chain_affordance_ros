# CLAUDE.md
This file provides guidance to Claude Code when working with this repository.

## Overview
A robot-agnostic ROS2 interface for the **Closed-Chain Affordance (CCA) planner** — a motion planning library for planning joint trajectories for manipulation tasks described as linear, rotational, or screw motions. For detailed project overview, usage examples, and code tutorial, see [README.md](README.md).

## Additional Guidance

### Packages
- `cca_ros` — ROS wrapper of the CCA library (core package)
- `cca_ros_msgs` — ROS message, service, and action definitions
- `cca_ros_util` — Utilities for working with the CCA planner (e.g. logging planning requests/results)
- `cca_ros_action` — CCA ROS as an action server and client wrapper
- `cca_ros_behavior` — CCA ROS as a BehaviorTree.CPP `StatefulActionNode`
- `cca_ros_behavior_util` — Utility behavior nodes (e.g. `LogReqToFile`)
- `cca_ros_features` — Higher-level feature library built on `cca_ros` (e.g. `getAffordativeGraspPose`)
- `cca_ros_behavior_features` — `cca_ros_features` functions wrapped as BehaviorTree.CPP nodes
- `cca_ros_rviz_plugin` — Interactive RViz plugin for visual planning
- `cca_ros_val_and_viz` — Trajectory validation (joint limits + self-collision) and visualization server
- `robot_state_recorder` — Records joint trajectories for analysis
- `ros_cpp_util` — General ROS/C++ utilities; no CCA dependency

### Architecture

### Core planning flow (`cca_ros`)
`CcaRos` is a `rclcpp::Node` subclass. Callers instantiate it, spin it, then call `.plan()` with one or more `PlanningRequest`s using a **request–response model**:
- `PlanningRequest` — specifies `planning_group`, `task_description`, `start_state`, `execute_trajectory`, `time_step`, etc.
- `PlanningResponse` — contains `result.success`, `result.joint_trajectory`, `result.cca_result`, and a shared `status` pointer

A `std::vector<PlanningRequest>` can be passed to plan a sequence of tasks into one stitched trajectory.

### Planning groups
A robot may have multiple planning groups (e.g. `arm`, `mobile_body_and_arm`). Each group has its own kinematic chain, joint names, reference frame, tool frame, and execution action server names, stored in `planning_group_info_map_` and populated from ROS parameters at startup. Each `PlanningRequest` specifies its `planning_group`.

### Validation and visualization (`cca_ros_val_and_viz`)
After planning, `CcaRos` calls the `/cca_ros_val_and_viz` service (MoveIt-based) to validate joint limits and collision, and to visualize the trajectory in RViz. The service is fully stateless — all needed info is in the request. See `CcaRosValAndViz.srv` for the full definition.

### Coding Conventions

#### General
- Each library package uses the `<package_name>` namespace, with headers in `include/<package_name>/` and implementations in `src/<package_name>/`
- Both files begin with the author header: `////...// Author : Crasun Jans`
- Private member variables and `static constexpr` constants: `snake_case_` (trailing underscore, e.g. `node_`)
- Member functions: `camelCase()`
- Header guards: `<FILENAME>_HPP_`
- Put all includes in the header; implementation files only include their corresponding header
- Use `std::optional<T>` as the return type for returning functions that may not produce a result.

#### Adding a library
1. `include/<package_name>/<n>.hpp` — declare and document with Doxygen
2. `src/<package_name>/<n>.cpp` — implement; include only the corresponding header
3. Register `.cpp` in `CMakeLists.txt` under `add_library()`
4. Add ROS dependencies to `ament_target_dependencies()` (with `PUBLIC`), `ament_export_dependencies()`, and `package.xml`
5. Add non-ROS dependencies to `target_link_libraries()` (with `PUBLIC`)
6. Only list dependencies that are directly used in the package's own headers or source — do not re-list dependencies that are already transitively available through another dependency

#### BehaviorTree node conventions
- The `rclcpp::Node::SharedPtr` is provided via the blackboard key `"node"`, not as an input port
- Default to `StatefulActionNode` over `SyncActionNode` unless explicitly stated otherwise
- Required input ports: use `getInput<T>()` and throw `BT::RuntimeError` on missing/invalid values
- Complex types passed as `std::shared_ptr<T>` to avoid expensive copying (e.g. `geometry_msgs::msg::PoseStamped`)
- When a feature function exists in `cca_ros_features`, the BehaviorTree node in `cca_ros_behavior_features` should be a thin wrapper: unpack ports, call the feature function, handle the result

##### StatefulActionNode lifecycle
- `onStart()` — called once; reads ports, allocates resources, dispatches async work; returns `RUNNING` or `FAILURE`
- `onRunning()` — called every tick while `RUNNING`; checks progress; returns `RUNNING`, `SUCCESS`, or `FAILURE`
- `onHalted()` — called on external halt; cancels pending async operations and releases resources

### Diff Constraints
When modifying existing files, keep the diff minimal and mechanical so it is easy to review. Only make changes required for the new functionality or bug fix—no unrelated edits.

Preserve exactly (byte-for-byte unless absolutely necessary):
- All comments and docstrings
- All variable and function names
- All error messages and log strings
- All formatting and indentation (including blank lines and brace placement)

Do not:
- Reformat, re-indent, or "clean up" code
- Rename symbols unless required for correctness
- Add or remove blank lines unnecessarily
- Introduce trailing whitespace

If code is moved, it must appear identical in the new location.
Formatting-only changes should never appear in the diff.
