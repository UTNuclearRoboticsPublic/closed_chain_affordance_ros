///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner_ros.hpp
//      Project   : cc_affordance_planner_ros
//      Created   : Spring 2024
//      Author    : Janak Panthi (Crasun Jans)
//      Copyright : Copyright© The University of Texas at Austin, 2014-2026.
//      All rights reserved.
//
//      All files within this directory are subject to the following, unless
//      an alternative license is explicitly included within the text of
//      each file.
//
//      This software and documentation constitute an unpublished work
//      and contain valuable trade secrets and proprietary information
//      belonging to the University. None of the foregoing material may be
//      copied or duplicated or disclosed without the express, written
//      permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//      AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//      INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//      PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//      THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//      NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//      THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//      University be liable for incidental, special, indirect, direct or
//      consequential damages or loss of profits, interruption of business,
//      or related expenses which may arise from use of software or
//      documentation, including but not limited to those resulting from
//      defects in software and/or documentation, or loss or inaccuracy of
//      data of any kind.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef CC_AFFORDANCE_PLANNER_ROS_HPP
#define CC_AFFORDANCE_PLANNER_ROS_HPP

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_listener.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <chrono>
#include <moveit_plan_and_viz_msgs/srv/move_it_plan_and_viz.hpp>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

namespace cc_affordance_planner_ros
{

/**
 * @brief Enum indicating the status of the CC Affordance Planner during execution.
 */
enum Status
{
    PROCESSING,
    SUCCEEDED,
    FAILED,
    UNKNOWN
};

/**
 * @brief Class representing the CC Affordance Planner node in ROS.
 * This class manages the process of planning, visualizing, and executing trajectories
 * for robot affordances using closed-chain kinematics.
 */
class CcAffordancePlannerRos : public rclcpp::Node
{
  public:
    // Type aliases
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveItPlanAndViz = moveit_plan_and_viz_msgs::srv::MoveItPlanAndViz;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using JointState = sensor_msgs::msg::JointState;

    /**
     * @brief Constructs a CcAffordancePlannerRos node.
     *
     * @param node_name Name of the ROS node.
     * @param options Options for the node, e.g., parameter overrides.
     */
    explicit CcAffordancePlannerRos(const std::string &node_name, const rclcpp::NodeOptions &options);

    /**
     * @brief Runs the CC Affordance planner for a single task, visualizes the robot joint trajectory, and executes it
     * on the robot.
     *
     * The planning is done from the current state of the robot unless a robot start configuration is passed.
     * Additionally, a pointer to the status of the planner can be provided to get updates on the process.
     *
     * @param planner_config Configuration settings for the CC Affordance planner. The struct includes:
     * - **accuracy**: Defines the threshold for the affordance goal. For example, if the goal is to rotate 5 radians
     * with 10% accuracy, the threshold would be set to 0.1, allowing a result within 5 ± 0.5 radians.
     * - **ik_max_itr**: (Advanced) Maximum iterations for the closed-chain inverse kinematics solver. Default is 200.
     * - **update_method**: (Advanced) Specifies the update method to use for solving the inverse kinematics problem.
     * Options are:
     *   - `INVERSE`: Uses the inverse of the Jacobian.
     *   - `TRANSPOSE`: Uses the transpose of the Jacobian.
     *   - `BEST`: Automatically selects the best method based on context (default).
     * - **closure_error_threshold_ang**: (Advanced) Angular error threshold for the closed-chain mechanism. Default is
     * 1e-4 radians.
     * - **closure_error_threshold_lin**: (Advanced) Linear error threshold for the closed-chain mechanism. Default is
     * 1e-5 meters.
     *
     * @param taskDescription Description of the task to plan for, structured as:
     * - **motion_type**: Specifies the type of motion, with possible values:
     *   - `cc_affordance_planner::APPROACH`: Cartesian approach motion.
     *   - `cc_affordance_planner::AFFORDANCE`: Affordance motion (default).
     * - **affordance_info**: Describes the affordance with the following fields:
     *   - **type**: Type of affordance (e.g., rotation, translation).
     *   - **axis**: The screw axis for the affordance motion (Eigen::Vector3d).
     *   - **location**: The point or location associated with the screw axis (Eigen::Vector3d).
     *   - **location_frame**: (Optional) Name of the reference frame for the affordance location. If specified, the
     * planner will attempt to find the affordance location using TF.
     * - **trajectory_density**: Specifies the density of the trajectory. This is the number of points along the
     * trajectory from the start to the goal. For instance, a goal of 0.5 radians can have 5 trajectory points spaced by
     * 0.1 radians.
     * - **goal**: The specific goal to achieve, which can be one or more of the following:
     *   - **affordance**: The final value of the affordance.
     *   - **ee_orientation**: Desired end-effector orientation (Eigen::Quaterniond).
     *   - **grasp_pose**: Cartesian goal for the end-effector to achieve during approach motion. Not relevant for
     * AFFORDANCE motion.
     *   - **gripper**: Desired state of the gripper specified as a joint value.
     * - **vir_screw_order**: (Optional) Specifies the virtual screw order for the closed-chain model. Default is
     * `affordance_util::VirtualScrewOrder::XYZ`, which assumes rotational freedom around the x, y, and z axes in order.
     *
     * @param status (Optional) Pointer to the cc_affordance_planner_ros::Status indicating the current status of the
     * planner. The status is set to `PROCESSING` during the planning phase, and it will be updated to either
     * `SUCCEEDED` or `FAILED` based on the result.
     * @param robotStartConfig (Optional) Initial configuration of the robot as an Eigen::VectorXd vector. If this is
     * not specified, the planner will use the current robot configuration obtained from the joint states topic.
     *
     * @return bool True if the planning and execution are successful; false otherwise.
     */
    bool run_cc_affordance_planner(
        const cc_affordance_planner::PlannerConfig &planner_config,
        const cc_affordance_planner::TaskDescription &taskDescription,
        const std::shared_ptr<Status> status =
            std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN),
        const Eigen::VectorXd &robotStartConfig = Eigen::VectorXd());

    /**
     * @brief Runs the CC Affordance planner for multiple tasks producing a single joint trajectory. Visualizes and
     * executes it on the robot.
     *
     * The planning is done sequentially for each task from the current robot state or optionally from a specified robot
     * start configuration. A pointer to track the status of the overall planning and execution can also be passed.
     *
     * @param planner_configs Vector of configuration settings for the CC Affordance planner. Each config struct
     * contains:
     * - **accuracy**: Defines the threshold for the affordance goal. For example, if the goal is to rotate 5 radians
     * with 10% accuracy, the threshold would be set to 0.1, allowing a result within 5 ± 0.5 radians.
     * - **ik_max_itr**: (Advanced) Maximum iterations for the closed-chain inverse kinematics solver. Default is 200.
     * - **update_method**: (Advanced) Specifies the update method to use for solving the inverse kinematics problem.
     * Options are:
     *   - `INVERSE`: Uses the inverse of the Jacobian.
     *   - `TRANSPOSE`: Uses the transpose of the Jacobian.
     *   - `BEST`: Automatically selects the best method based on context (default).
     * - **closure_error_threshold_ang**: (Advanced) Angular error threshold for the closed-chain mechanism. Default is
     * 1e-4 radians.
     * - **closure_error_threshold_lin**: (Advanced) Linear error threshold for the closed-chain mechanism. Default is
     * 1e-5 meters.
     *
     * @param task_descriptions Vector of task descriptions, where each task describes a different goal for the planner.
     * The task struct contains:
     * - **motion_type**: Specifies the type of motion, with possible values:
     *   - `cc_affordance_planner::APPROACH`: Cartesian approach motion.
     *   - `cc_affordance_planner::AFFORDANCE`: Affordance motion (default).
     * - **affordance_info**: Describes the affordance with the following fields:
     *   - **type**: Type of affordance (e.g., rotation, translation).
     *   - **axis**: The screw axis for the affordance motion (Eigen::Vector3d).
     *   - **location**: The point or location associated with the screw axis (Eigen::Vector3d).
     *   - **location_frame**: (Optional) Name of the reference frame for the affordance location. If specified, the
     * planner will attempt to find the affordance location using TF.
     * - **trajectory_density**: Specifies the density of the trajectory. This is the number of points along the
     * trajectory from the start to the goal. For instance, a goal of 0.5 radians can have 5 trajectory points spaced by
     * 0.1 radians.
     * - **goal**: The specific goal to achieve, which can be one or more of the following:
     *   - **affordance**: The final value of the affordance.
     *   - **ee_orientation**: Desired end-effector orientation (Eigen::Quaterniond).
     *   - **grasp_pose**: Cartesian goal for the end-effector to achieve during approach motion. Not relevant for
     * AFFORDANCE motion.
     *   - **gripper**: Desired state of the gripper specified as a joint value.
     * - **vir_screw_order**: (Optional) Specifies the virtual screw order for the closed-chain model. Default is
     * `affordance_util::VirtualScrewOrder::XYZ`, which assumes rotational freedom around the x, y, and z axes in order.
     *
     * @param status (Optional) Pointer to the cc_affordance_planner_ros::Status indicating the result of the planning
     * and execution. Statuses are:
     * - `PROCESSING`: The planner is currently working on the tasks.
     * - `SUCCEEDED`: All tasks were successfully planned, visualized, and executed.
     * - `FAILED`: One or more tasks failed during planning or execution.
     *
     * @param robotStartConfig (Optional) Specifies the initial configuration of the robot as an Eigen::VectorXd. If
     * this is not provided, the current robot configuration is retrieved from the joint states topic.
     *
     * @return bool True if all tasks were successfully planned and executed, false otherwise.
     */
    bool run_cc_affordance_planner(
        const std::vector<cc_affordance_planner::PlannerConfig> &planner_configs,
        const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions,
        const std::shared_ptr<Status> status =
            std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN),
        const Eigen::VectorXd &robotStartConfig = Eigen::VectorXd());

  private:
    std::shared_ptr<Status> status_{nullptr}; ///< Current status of planning and execution
    std::mutex status_mutex_;                 ///< Mutex to protect _status access
    rclcpp::Logger node_logger_;              ///< Node-specific logger
    std::string plan_and_viz_ss_name_;        ///< Name of the plan and visualization server
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        robot_traj_execution_client_; ///< Client for executing robot trajectory
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        gripper_traj_execution_client_;                               ///< Client for executing gripper trajectory
    rclcpp::Client<MoveItPlanAndViz>::SharedPtr plan_and_viz_client_; ///< Client for visualizing the planned trajectory
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;    ///< Subscriber for joint states
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                      ///< TF2 buffer for transformation lookup
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; ///< TF2 transform listener

    // Robot ROS setup data
    std::string robot_traj_execution_as_name_;   ///< Action server name for robot trajectory execution
    std::string gripper_traj_execution_as_name_; ///< Action server name for gripper trajectory execution
    std::string planning_group_;                 ///< MoveIt planning group for visualization
    std::string robot_description_parameter_;    ///< Parameter for the robot description
    std::string rviz_fixed_frame_;               ///< Fixed frame for RViz visualization

    // Robot data
    Eigen::MatrixXd robot_slist_;                  ///< Screw axes list for the robot
    std::vector<std::string> robot_joint_names_;   ///< Robot joint names
    std::vector<std::string> gripper_joint_names_; ///< Gripper joint names
    Eigen::Matrix<double, 4, 4> M_;                ///< Home configuration matrix for the robot
    std::string ref_frame_;                        ///< Reference frame for transformations
    std::string tool_frame_;                       ///< Tool frame for the robot's end-effector

    affordance_util_ros::JointTrajPoint robot_joint_states_;   ///< Processed and ordered robot joint states
    affordance_util_ros::JointTrajPoint gripper_joint_states_; ///< Processed and ordered gripper joint states

    /**
     * @brief Validates a single task description for the CC Affordance Planner ROS node.
     *
     * @param task_description The task description to validate.
     * @throws std::invalid_argument If the task description has issues.
     */
    void validate_input_(const cc_affordance_planner::TaskDescription &task_description);

    /**
     * @brief Validates multiple task descriptions and planner configurations for the CC Affordance Planner ROS node.
     *
     * @param planner_configs The vector of planner configurations to validate.
     * @param task_descriptions The vector of task descriptions to validate.
     * @throws std::invalid_argument If the task descriptions have issues.
     */
    void validate_input_(const std::vector<cc_affordance_planner::PlannerConfig> &planner_configs,
                         const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions);

    /**
     * @brief Returns the path to the CC Affordance robot description YAML file.
     *
     * @param robot_name Name of the robot package (e.g., "spot" for "cca_spot").
     * @return Full path to the YAML file containing the robot description.
     */
    static std::string get_cc_affordance_robot_description_(const std::string &robot_name);

    /**
     * @brief Callback function for processing joint state updates.
     *
     * @param msg Incoming joint state message.
     */
    void joint_states_cb_(const JointState::SharedPtr msg);

    /**
     * @brief Retrieves the start joint states of the robot for the affordance.
     *
     * @return Pair of Eigen::VectorXd containing robot and gripper start configurations.
     */
    std::pair<Eigen::VectorXd, Eigen::VectorXd> get_aff_start_joint_states_();

    /**
     * @brief Visualizes and executes the planned trajectory.
     *
     * @param trajectory Vector of joint states representing the planned trajectory.
     * @param w_aff Affordance screw axis.
     * @param q_aff Affordance location.
     * @param includes_gripper_trajectory Whether the gripper trajectory is included.
     * @return True if successful, false otherwise.
     */
    bool visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory, const Eigen::VectorXd &w_aff,
                                           const Eigen::VectorXd &q_aff, const bool includes_gripper_trajectory);

    /**
     * @brief Executes the given trajectory on the robot.
     *
     * @param traj_execution_client Action client for executing the trajectory.
     * @param send_goal_options Options for sending the trajectory execution goal.
     * @param traj_execution_as_name Name of the action server.
     * @param joint_names List of joint names.
     * @param trajectory Vector of joint states representing the trajectory.
     * @return True if successful, false otherwise.
     */
    bool execute_trajectory_(rclcpp_action::Client<FollowJointTrajectory>::SharedPtr &traj_execution_client,
                             rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options,
                             const std::string &traj_execution_as_name, const std::vector<std::string> &joint_names,
                             const std::vector<Eigen::VectorXd> &trajectory);

    /**
     * @brief Callback for handling the result of robot trajectory execution.
     *
     * @param result Result from the FollowJointTrajectory action server.
     */
    void robot_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    /**
     * @brief Callback for handling the goal response of robot trajectory execution.
     *
     * @param goal_handle Shared pointer to the goal handle.
     */
    void robot_traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);

    /**
     * @brief Callback for handling the result of gripper trajectory execution.
     *
     * @param result Result from the FollowJointTrajectory action server.
     */
    void gripper_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    /**
     * @brief Callback for handling the goal response of gripper trajectory execution.
     *
     * @param goal_handle Shared pointer to the goal handle.
     */
    void gripper_traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);

    /**
     * @brief Analyzes the result code returned by an action server and updates the cc_affordance_planner_ros status
     * accordingly.
     *
     * @param result_code The result code returned by a ROS action server
     *
     * @param as_name The name of the action server (either for the robot or the gripper), used for logging.
     *
     * @note The function ensures that if any of the action results in failure, the final status will be `FAILED`,
     * and a successful action will not override a previous failure.
     */
    void analyze_as_result_(const rclcpp_action::ResultCode &result_code, const std::string &as_name);
};

} // namespace cc_affordance_planner_ros

#endif // CC_AFFORDANCE_PLANNER_ROS_HPP
