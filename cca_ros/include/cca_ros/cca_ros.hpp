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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cc_affordance_planner/cc_affordance_planner_util.hpp>
#include <cca_ros_msgs/srv/cca_ros_val_and_viz.hpp>
#include <chrono>
#include <cmath>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ros_cpp_util/ros_cpp_util.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <stop_token>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <unordered_map>

namespace cca_ros
{
using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FollowJointTrajectoryGoal = control_msgs::action::FollowJointTrajectory_Goal;

/**
* @brief Struct containing execution action clients.
*/
struct ExecutionActionClients{
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr robot{nullptr}; /**< Client for executing robot trajectory.*/
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper{nullptr}; /**< Client for executing gripper trajectory. */
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr robot_and_gripper{nullptr};     /**< Client for executing robot and gripper trajectory together. */
};

/**
* @brief Struct containing execution action server names.
*/
struct ExecutionActionServerNames{
    std::string robot; /**< Action server name for robot trajectory execution. */
    std::string gripper; /**< Action server name for gripper trajectory execution. */
    std::string robot_and_gripper; /**< Action server name for combined robot and gripper trajectory execution. */
};

/**
* @brief Struct containing planning group information.
*/
struct PlanningGroupInfo{
    affordance_util::RobotConfig robot_config; /**< Robot configuration details. */
    ExecutionActionServerNames ex_as_names;    /**< Names of execution action servers. */
    ExecutionActionClients ex_clients;         /**< Execution action clients. */
};

/**
 * @brief Struct containing the kinematic state of a robot.
 */
struct KinematicState
{
    Eigen::VectorXd robot; /**< Joint positions of the robot. */
    double gripper;        /**< Position of the gripper. */
};

/**
 * @brief Enum indicating the status of the CC Affordance Planner during
 * execution.
 */
enum Status
{
    PROCESSING, /**< Planning or execution is in progress. */
    SUCCEEDED,  /**< Operation completed successfully. */
    FAILED,     /**< Operation failed. */
    UNKNOWN     /**< Status is unknown. */
};

/**
 * @brief Struct to hold timesteps for the trajectory.
 */
struct TrajectoryTimeStep
{
    double robot = 0.3;             /**< Time step for robot trajectory (seconds). */
    double gripper = 0.3;           /**< Time step for gripper trajectory (seconds). */
    double robot_and_gripper = 0.3; /**< Time step for combined robot and gripper
                                       trajectory (seconds). */
};

/**
 * @brief Struct containing planning request for the CCA ROS planner.
 */
struct PlanningRequest
{
    std::string planning_group; /**< Name of the planning group to use. */
    cc_affordance_planner::PlannerConfig planner_config = cc_affordance_planner::PlannerConfig(); /**< Configuration for
                                                                                                     the planner. */
    cc_affordance_planner::TaskDescription task_description; /**< Description of the task to plan. */
    KinematicState start_state =
        KinematicState{Eigen::VectorXd(), std::numeric_limits<double>::quiet_NaN()}; /**< Initial kinematic state.
                                                                                      */
    bool execute_trajectory = false; /**< Whether to execute the planned trajectory. */
    bool execute_partial_trajectory = false; /**< Whether to execute partially planned trajectory. */
    std::chrono::seconds execution_timeout{60}; /**< Timeout for trajectory execution. CcaRos returns failure sends a cancel request to the trajectory execution server if this timeout is exceeded. */ 
    TrajectoryTimeStep time_step;    /**< Time steps for the trajectory. */
};

/**
 * @brief Struct containing planning response for the CCA ROS planner.
 */
struct PlanningResponse
{
    /**
     * @brief Nested struct for planning result details.
     */
    struct PlanningResult
    {
        bool success = false;                            /**< Whether planning was successful. */
        FollowJointTrajectoryGoal joint_trajectory;      /**< Solved joint trajectory message. */
        cc_affordance_planner::PlannerResult cca_result; /**< Raw result from the CCA planner. */
    };

    std::shared_ptr<Status> status =
        std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN); /**< Current status of the operation. */
    PlanningResult result;                                           /**< Result details of the planning. */
};

/**
 * @brief Struct for goal messages for robot, gripper, and combined
 * trajectories.
 */
struct GoalMsg
{
    FollowJointTrajectoryGoal robot;             /**< Goal for robot trajectory. */
    FollowJointTrajectoryGoal gripper;           /**< Goal for gripper trajectory. */
    FollowJointTrajectoryGoal robot_and_gripper; /**< Goal for combined robot and gripper trajectory. */
};

/**
 * @brief Class representing the CC Affordance Planner node in ROS.
 * This class manages the process of planning, visualizing, and executing
 * trajectories for robot affordances using closed-chain kinematics.
 */
class CcaRos 
{
  public:
    // Type aliases
    using CcaRosValAndViz = cca_ros_msgs::srv::CcaRosValAndViz;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using JointState = sensor_msgs::msg::JointState;

    /**
     * @brief Constructs a CcaRos node.
     * @param node_name Name of the ROS node.
     * @param options Options for the node, e.g., parameter overrides.
     */
    explicit CcaRos(const std::string &node_name, const rclcpp::NodeOptions &options);

    /**
     * @brief Constructs a CcaRos node.
     * @param node Shared pointer to an existing ROS node.
     */
    explicit CcaRos(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief Destructs a CcaRos node.
     */
    ~CcaRos();

    /**
     * @brief Plans and optionally executes a single task based on the provided
     * planning request.
     *
     * This function takes a single PlanningRequest, which includes the task
     * description, and optionally, planner configuration, initial kinematic
     * state, execution flag, and time delta for trajectory points.
     *
     * The PlanningRequest struct contains:
     * - task_description: Details of the task (e.q. affordance info, goals, etc.)
     * - execute_trajectory: Boolean flag indicating whether to execute the
     * planned trajectory.
     * - planner_config: Optional, configuration parameters for the planner, such
     * as accuracy, max IK iterations, etc.
     * - start_state: Optional, initial start configuration to plan from. Default
     * behavior is to automatically plan from current robot state.
     * - time_step: Struct specifying time deltas between points in the generated
     * joint trajectory.
     *
     * The returned PlanningResponse includes:
     * - status: A shared pointer to the current status (e.g., PROCESSING,
     * SUCCEEDED, FAILED, UNKNOWN).
     * - result: A struct with success flag, the solved joint trajectory message,
     * and raw planner result.
     *
     * @param planning_request The request containing necessary information for
     * planning.
     * @return PlanningResponse with the outcome of the planning and optional
     * execution.
     */
    cca_ros::PlanningResponse plan(const cca_ros::PlanningRequest &planning_request);

    /**
     * @brief Plans and optionally executes a sequence of tasks based on the
     * provided planning request vector.
     *
     * This function takes a PlanningRequest vector for a sequence of tasks to be
     * planned together. Each PlanningRequest includes the task description, and
     * optionally, planner configuration, initial kinematic state, execution flag,
     * and time delta for trajectory points. Execution of the trajectory is based
     * on the execute_trajectory flag of the first request in the vector.
     *
     * The PlanningRequest struct contains:
     * - task_description: Details of the task (e.q. affordance info, goals, etc.)
     * - execute_trajectory: Boolean flag indicating whether to execute the
     * planned trajectory.
     * - planner_config: Optional, configuration parameters for the planner, such
     * as accuracy, max IK iterations, etc.
     * - start_state: Optional, initial start configuration to plan from. Default
     * behavior is to automatically plan from current robot state.
     * - time_step: Struct specifying time deltas between points in the generated
     * joint trajectory.
     *
     * The returned PlanningResponse includes:
     * - status: A shared pointer to the current status (e.g., PROCESSING,
     * SUCCEEDED, FAILED, UNKNOWN).
     * - result: A struct with success flag, the solved joint trajectory message,
     * and raw planner result.
     *
     * @param planning_request The request containing necessary information for
     * planning.
     * @return PlanningResponse with the outcome of the planning and optional
     * execution.
     */
    cca_ros::PlanningResponse plan(const std::vector<cca_ros::PlanningRequest> &planning_requests);

    /**
     * @brief Cancels ongoing trajectory execution.
     */
    void cancel_execution();

    /**
    * @brief Retrieves the Cca planning group information map from ROS parameters. Note that this function does not initialize execution action clients in the 
    * planning group info. That should be done outside using the CcaRos node context.
    *
    * @param node_ptr Pointer to the ROS node.
    *
    * @return 
    */
    static std::unordered_map<std::string, PlanningGroupInfo> get_planning_group_info_map(rclcpp::Node* node_ptr);

  private:
    rclcpp::Node::SharedPtr node_; /**< Shared pointer to the ROS node. */
    std::unordered_map<std::string, cca_ros::PlanningGroupInfo> planning_group_info_map_; /**< Mapping of planning group names to their information. */
    constexpr static double tf_lookup_timeout_ = 1.5; /**< Wait until 1.5 secs for TF lookups */
    constexpr static std::chrono::seconds joint_states_read_timeout_{5}; /**< Timeout for reading joint states. */ 
    constexpr static std::chrono::seconds val_and_viz_ss_avail_wait_{1}; /**< How long to wait for the validation service to be available. */ 
    constexpr static std::chrono::seconds ex_as_avail_wait_{1}; /**< How long to wait for the execution action servers to be available. */ 
    constexpr static double traj_completion_threshold_ = 0.5; /**< Completion threshold for partial trajectory execution. Planner needs to generate at least 50%. */
    constexpr static double start_state_tolerance_ = 1e-1; /**< Tolerance for start state deviation check during execution. */
    std::shared_ptr<Status> status_{nullptr};                 /**< Current status of planning and execution. */
    std::shared_ptr<Status> robot_result_status_ = {
        nullptr}; /**< Current status of robot trajectory execution result. */
    std::shared_ptr<Status> gripper_result_status_ = {
        nullptr};                       /**< Current status of gripper trajectory execution result. */
    std::jthread result_status_thread_; /**< Thread to check the status of robot
                                           and gripper trajectory results. */
    std::mutex status_mutex_;           /**< Mutex to protect access to status_. */
    std::mutex joint_states_mutex_;      /**< Mutex to protect access to joint states. */
    std::condition_variable joint_states_cv_; /**< Condition variable to signal availability of joint states. */
    rclcpp::Logger node_logger_;        /**< Node-specific logger. */
    std::string val_and_viz_ss_name_;           /**< Name of the plan and visualization server. */
    ExecutionActionClients ex_clients_; /**< Action clients for trajectory execution. */
    rclcpp::Client<CcaRosValAndViz>::SharedPtr val_and_viz_client_; /**< Client for visualizing the planned trajectory. */
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;     /**< Subscriber for joint states. */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                       /**< TF2 buffer for transformation lookup. */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; /**< TF2 transform listener. */

    // Robot ROS setup data
    cca_ros::ExecutionActionServerNames ex_as_names_; /**< Current action server names for execution. */

    // Robot data
    Eigen::MatrixXd robot_slist_;                  /**< Screw axes list for the robot. */
    std::vector<std::string> robot_joint_names_;   /**< Robot joint names. */
    std::vector<std::string> gripper_joint_names_; /**< Gripper joint names. */
    Eigen::Matrix<double, 4, 4> M_;                /**< Home configuration matrix for the robot. */
    std::string ref_frame_;                        /**< Reference frame for transformations. */
    std::string tool_frame_;                       /**< Tool frame for the robot's end-effector. */
    std::string planning_group_;                    /**< Current planning group name. */


    ros_cpp_util::JointTrajPoint robot_joint_states_;   /**< Processed and ordered robot joint states. */
    ros_cpp_util::JointTrajPoint gripper_joint_states_; /**< Processed and ordered gripper joint states. */

    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr>
        unified_gh_future_; /**< Goal handle future for the unified trajectory
                               executor. */
    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> robot_gh_future_; /**< Goal handle future for the
                                                                                        robot trajectory executor. */
    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr>
        gripper_gh_future_; /**< Goal handle future for the gripper trajectory
                               executor. */

    /**
     * @brief Validates multiple planning requests.
     * @param reqs Vector of planning requests to validate.
     * @throws std::invalid_argument If validation fails.
     */
    void validate_input_(const std::vector<cca_ros::PlanningRequest> &reqs);

    /**
     * @brief Callback function for processing joint state updates.
     * @param msg Incoming joint state message.
     */
    void joint_states_cb_(const JointState::SharedPtr msg);

    /**
     * @brief Retrieves the joint states of the robot and gripper.
     * @return KinematicState containing robot and gripper state.
     */
    KinematicState read_joint_states_();

    /**
     * @brief Validates and visualizes a given trajectory.
     * @param goal FollowJointTrajectory goal for the trajectory.
     * @param cartesian_trajectory Corresponding Cartesian trajectory.
     * @param task_descriptions Descriptions of tasks for visualization.
     * @return Shared pointer to the visualization service response.
     */
    cca_ros_msgs::srv::CcaRosValAndViz::Response::SharedPtr validate_and_visualize_(
        const FollowJointTrajectoryGoal &goal, const std::vector<geometry_msgs::msg::Pose> &cartesian_trajectory,
        const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions);

    /**
     * @brief Executes the planned trajectory.
     * @param goal_msg Goal messages for robot, gripper, and combined
     * trajectories.
     * @param includes_gripper_trajectory Whether gripper trajectory is included.
     * @param execution_timeout Timeout for trajectory execution. If the trajectory execution server does not return a result within this timeout, CcaRos will send a cancel request and return FAILURE status.
     * @return True if execution succeeds, false otherwise.
     */
    bool execute_(const cca_ros::GoalMsg &goal_msg, bool includes_gripper_trajectory, const std::chrono::seconds& execution_timeout);

    /**
     * @brief Sends an execution goal to the specified action server.
     * @param traj_execution_client Action client for trajectory execution.
     * @param send_goal_options Options for sending the goal.
     * @param traj_execution_as_name Name of the action server.
     * @param goal FollowJointTrajectory goal message.
     * @param goal_handle_future Future for the goal handle.
     * @return True if goal is sent successfully, false otherwise.
     */
    bool send_execution_goal_(rclcpp_action::Client<FollowJointTrajectory>::SharedPtr &traj_execution_client,
                              rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options,
                              const std::string &traj_execution_as_name, const FollowJointTrajectoryGoal &goal,
                              std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> &goal_handle_future);

    /**
     * @brief Callback for handling the result of robot trajectory execution.
     * @param result Result from the FollowJointTrajectory action server.
     */
    void robot_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    /**
     * @brief Callback for handling the goal response of robot trajectory
     * execution.
     * @param goal_handle Shared pointer to the goal handle.
     */
    void robot_traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);

    /**
     * @brief Callback for handling the result of gripper trajectory execution.
     * @param result Result from the FollowJointTrajectory action server.
     */
    void gripper_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    /**
     * @brief Callback for handling the goal response of gripper trajectory
     * execution.
     * @param goal_handle Shared pointer to the goal handle.
     */
    void gripper_traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);

    /**
     * @brief Analyzes the result code from an action server and returns the
     * corresponding status.
     * @param result_code Result code from the action server.
     * @param as_name Name of the action server for logging.
     * @return Corresponding Status enum value.
     */
    Status analyze_as_result_(const rclcpp_action::ResultCode &result_code, const std::string &as_name);

    /**
     * @brief Checks status(es) for execution result(s) from action server(s) and
     * updates the node status.
     * @param st Stop token to handle thread cancellation.
     * @param includes_gripper_trajectory Whether gripper trajectory is included in the current task.
     * @param execution_timeout Timeout for waiting for execution results before canceling execution request and returning FAILURE.
     */
    void check_execution_result_status_(std::stop_token st, bool includes_gripper_trajectory, const std::chrono::seconds& execution_timeout);

    /**
     * @brief Creates goal messages for robot, gripper, and combined trajectories.
     * @param trajectory Joint trajectory data.
     * @param includes_gripper_trajectory Whether gripper trajectory is included.
     * @param time_step Time steps for the trajectory.
     * @return GoalMsg containing the goal messages.
     */
    cca_ros::GoalMsg create_goal_msg_(const std::vector<Eigen::VectorXd> &trajectory, bool includes_gripper_trajectory,
                                      const TrajectoryTimeStep &time_step);

    /**
     * @brief Computes the Cartesian trajectory from a joint trajectory.
     * @param trajectory Joint trajectory.
     * @return Vector of Cartesian poses.
     */
    std::vector<geometry_msgs::msg::Pose> compute_cartesian_trajectory_(const std::vector<Eigen::VectorXd> &trajectory);

    /**
     * @brief Initializes action clients based on available parameters.
     * @param ex_as_names Execution action server names.
     */
    cca_ros::ExecutionActionClients initialize_action_clients_(const cca_ros::ExecutionActionServerNames& ex_as_names);
};

} // namespace cca_ros

#endif // CCA_ROS_HPP
