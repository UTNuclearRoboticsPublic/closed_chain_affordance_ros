///////////////////////////////////////////////////////////////////////////////
//      Title     : cca_ros.hpp
//      Project   : cca_ros
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

#ifndef CCA_ROS_HPP
#define CCA_ROS_HPP

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
#include <cca_ros_viz_msgs/srv/cca_ros_viz.hpp>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <thread>

namespace cca_ros
{
using namespace std::chrono_literals;
using FollowJointTrajectoryGoal = control_msgs::action::FollowJointTrajectory_Goal;

/**
 * @brief Struct containing the kinematic state of a robot
 */
struct KinematicState
{
    Eigen::VectorXd robot;
    double gripper;
};

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
 * @brief Struct containing planning request for the CCA ROS planner
 */
struct PlanningRequest
{
    cc_affordance_planner::PlannerConfig planner_config = cc_affordance_planner::PlannerConfig();
    cc_affordance_planner::TaskDescription task_description;
    KinematicState start_state = KinematicState{Eigen::VectorXd(), std::numeric_limits<double>::quiet_NaN()};
    std::shared_ptr<Status> status = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);
};

/**
 * @brief Struct containing planning requests for the CCA ROS planner
 */
struct PlanningRequests
{
    std::vector<cc_affordance_planner::PlannerConfig> planner_config;
    std::vector<cc_affordance_planner::TaskDescription> task_description;
    KinematicState start_state = KinematicState{Eigen::VectorXd(), std::numeric_limits<double>::quiet_NaN()};
    std::shared_ptr<Status> status = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);
};

/**
 * @brief Class representing the CC Affordance Planner node in ROS.
 * This class manages the process of planning, visualizing, and executing trajectories
 * for robot affordances using closed-chain kinematics.
 */
class CcaRos : public rclcpp::Node
{
  public:
    // Type aliases
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using CcaRosViz = cca_ros_viz_msgs::srv::CcaRosViz;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using JointState = sensor_msgs::msg::JointState;

    // Variables
    bool visualize_trajectory;
    bool execute_trajectory;
    /**
     * @brief Constructs a CcaRos node.
     *
     * @param node_name Name of the ROS node.
     * @param options Options for the node, e.g., parameter overrides.
     * @param visualize_trajectory Whether to visualize the solved trajectory. Default is true.
     * @param execute_trajectory Whether to execute the solved trajectory on the robot. Default is false.
     */
    explicit CcaRos(const std::string &node_name, const rclcpp::NodeOptions &options, bool visualize_trajectory = true,
                    bool execute_trajectory = false);

    /**
     * @brief Cleans up and destructs a CcaRos node .
     */
    ~CcaRos();

    /**
     * @brief Runs the CC Affordance planner for a single task, visualizes the robot joint trajectory, and executes it
     * on the robot.
     *
     * The planning is done from the current state of the robot unless a robot start configuration is passed.
     * Additionally, a pointer to the status of the planner can be provided to get updates on the process.
     *
     * @param planning_request cca_ros::PlanningRequest containing planning information. See repo README for detailed
     * information about the struct members.
     *
     * @return bool True if the planning and execution are successful; false otherwise.
     */
    bool plan_visualize_and_execute(const cca_ros::PlanningRequest &planning_request);

    /**
     * @brief Runs the CCA planner for multiple tasks, producing a single joint trajectory.
     *        Visualizes and executes it on the robot.
     *
     * The planning is done sequentially for each task, starting from the current robot state or optionally
     * from a specified robot start configuration. A pointer to track the status of the overall planning and
     * execution can also be passed.
     *
     * @param planning_requests cca_ros::PlanningRequests containing planning information. See repo README for detailed
     * information about the struct members.
     *
     * @return bool `true` if all tasks were successfully planned and executed, `false` otherwise.
     */

    bool plan_visualize_and_execute(const cca_ros::PlanningRequests &planning_requests);

    /**
     * @brief Joins and cleans up status-checking thread
     */
    void cleanup_threads();

    /**
     * @brief Cancels trajectory execution on robot
     */
    void cancel_execution();

  private:
    std::shared_ptr<Status> status_{nullptr};                 ///< Current status of planning and execution
    std::shared_ptr<Status> robot_result_status_ = {nullptr}; ///< Current status of robot trajectory execution result
    std::shared_ptr<Status> gripper_result_status_ = {
        nullptr};                      ///< Current status of gripper trajectory execution result
    std::thread result_status_thread_; ///< Thread to check the status of robot and gripper trajectory results
    std::mutex status_mutex_;          ///< Mutex to protect access to status_
    rclcpp::Logger node_logger_;       ///< Node-specific logger
    std::string viz_ss_name_;          ///< Name of the plan and visualization server
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        robot_traj_execution_client_; ///< Client for executing robot trajectory
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        gripper_traj_execution_client_; ///< Client for executing gripper trajectory
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        robot_and_gripper_traj_execution_client_;     ///< Client for executing robot and gripper trajectory together
    rclcpp::Client<CcaRosViz>::SharedPtr viz_client_; ///< Client for visualizing the planned trajectory
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;     ///< Subscriber for joint states
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                       ///< TF2 buffer for transformation lookup
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; ///< TF2 transform listener

    // Robot ROS setup data
    std::string robot_traj_execution_as_name_;             ///< Action server name for robot trajectory execution
    std::string gripper_traj_execution_as_name_;           ///< Action server name for gripper trajectory execution
    std::string robot_and_gripper_traj_execution_as_name_; ///< Action server name for robot and gripper trajectory
                                                           ///< execution together

    // Robot data
    Eigen::MatrixXd robot_slist_;                  ///< Screw axes list for the robot
    std::vector<std::string> robot_joint_names_;   ///< Robot joint names
    std::vector<std::string> gripper_joint_names_; ///< Gripper joint names
    Eigen::Matrix<double, 4, 4> M_;                ///< Home configuration matrix for the robot
    std::string ref_frame_;                        ///< Reference frame for transformations
    std::string tool_frame_;                       ///< Tool frame for the robot's end-effector

    affordance_util_ros::JointTrajPoint robot_joint_states_;   ///< Processed and ordered robot joint states
    affordance_util_ros::JointTrajPoint gripper_joint_states_; ///< Processed and ordered gripper joint states

    bool unified_executor_available_ =
        false; ///< Indicates whether an action server is available to execute the robot and gripper trajectory together

    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr>
        unified_gh_future_; ///< Goal handle future for the unified trajectory executor
    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr>
        robot_gh_future_; ///< Goal handle future for the robot trajectory executor
    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr>
        gripper_gh_future_; /// Goal handle future for the gripper trajectory executor

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
     * @brief Retrieves the joint states of the robot and gripper.
     *
     * @return State struct containing robot and gripper state.
     */
    KinematicState read_joint_states_();

    /**
     * @brief Visualizes and executes the planned trajectory.
     *
     * @param trajectory Vector of joint states representing the planned trajectory.
     * @param w_aff Affordance screw axis.
     * @param q_aff Affordance location.
     * @return True if successful, false otherwise.
     */
    bool visualize_trajectory_(const FollowJointTrajectoryGoal &goal,
                               const std::vector<geometry_msgs::msg::Pose> &cartesian_trajectory,
                               const Eigen::VectorXd &w_aff, const Eigen::VectorXd &q_aff,
                               const std::optional<geometry_msgs::msg::Pose> &aff_ref_pose = std::nullopt);

    /**
     * @brief Executes the given trajectory on the robot.
     *
     * @param traj_execution_client Action client for executing the trajectory.
     * @param send_goal_options Options for sending the trajectory execution goal.
     * @param traj_execution_as_name Name of the action server.
     * @param goal follow_joint_trajectory goal message containing the trajectory to execute
     * @param goal_handle_future follow_joint_trajectory-type action goal handle
     * @return True if successful, false otherwise. Also, returns the goal handle by reference
     */
    bool execute_trajectory_(rclcpp_action::Client<FollowJointTrajectory>::SharedPtr &traj_execution_client,
                             rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options,
                             const std::string &traj_execution_as_name, const FollowJointTrajectoryGoal &goal,
                             std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> &goal_handle_future);

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
     * @brief Analyzes the result code returned by an action server and updates the cca_ros status
     * accordingly.
     *
     * @param result_code The result code returned by a ROS action server
     *
     * @param as_name The name of the action server (either for the robot or the gripper), used for logging.
     *
     * @return A result status
     */
    Status analyze_as_result_(const rclcpp_action::ResultCode &result_code, const std::string &as_name);
    /**
     * @brief Checks statuses for robot and gripper trajectory execution and sets node status based on them
     */
    void check_robot_and_gripper_result_status_();

    /**
     * @brief Creates follow joint trajectory messages for the robot and/or gripper trajectory visualization and
     * execution servers
     *
     * @param trajectory Bare trajectory containing robot and/or gripper trajectory
     * @param includes_gripper_trajectory Bool indicating whether the trajectory arg contains gripper trajectory
     *
     * @return Tuple or ROS follow_joint_trajectory goal messages for the robot, gripper, and robot and gripper
     * together. Robot msg is always returned. Other two are conditional.
     */
    std::tuple<FollowJointTrajectoryGoal, FollowJointTrajectoryGoal, FollowJointTrajectoryGoal> create_goal_msg_(
        const std::vector<Eigen::VectorXd> &trajectory, bool includes_gripper_trajectory);

    /**
     * @brief Given a robot joint trajectory computes the corresponding cartesian trajectory that the robot tool will
     * trace
     *
     * @param trajectory robot joint trajectory
     *
     * @return
     */
    std::vector<geometry_msgs::msg::Pose> compute_cartesian_trajectory_(const std::vector<Eigen::VectorXd> &trajectory);

    /**
     * @brief Initializes proper action clients in the constructor
     */
    void initialize_action_clients_();
};

} // namespace cca_ros

#endif // CCA_ROS_HPP
