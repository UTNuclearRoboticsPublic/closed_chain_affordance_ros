///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner_ros.hpp
//      Project   : cc_affordance_planner_ros
//      Created   : Spring 2024
//      Author    : Janak Panthi (Crasun Jans)
//      Copyright : Copyright© The University of Texas at Austin, 2014-2026. All
//      rights reserved.
//
//          All files within this directory are subject to the following, unless
//          an alternative license is explicitly included within the text of
//          each file.
//
//          This software and documentation constitute an unpublished work
//          and contain valuable trade secrets and proprietary information
//          belonging to the University. None of the foregoing material may be
//          copied or duplicated or disclosed without the express, written
//          permission of the University. THE UNIVERSITY EXPRESSLY DISCLAIMS ANY
//          AND ALL WARRANTIES CONCERNING THIS SOFTWARE AND DOCUMENTATION,
//          INCLUDING ANY WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
//          PARTICULAR PURPOSE, AND WARRANTIES OF PERFORMANCE, AND ANY WARRANTY
//          THAT MIGHT OTHERWISE ARISE FROM COURSE OF DEALING OR USAGE OF TRADE.
//          NO WARRANTY IS EITHER EXPRESS OR IMPLIED WITH RESPECT TO THE USE OF
//          THE SOFTWARE OR DOCUMENTATION. Under no circumstances shall the
//          University be liable for incidental, special, indirect, direct or
//          consequential damages or loss of profits, interruption of business,
//          or related expenses which may arise from use of software or
//          documentation, including but not limited to those resulting from
//          defects in software and/or documentation, or loss or inaccuracy of
//          data of any kind.
//
///////////////////////////////////////////////////////////////////////////////
#ifndef CC_AFFORDANCE_PLANNER_ROS
#define CC_AFFORDANCE_PLANNER_ROS

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
#include <cmath>
#include <cstdlib>
#include <future>
#include <moveit_plan_and_viz_msgs/srv/move_it_plan_and_viz.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

namespace cc_affordance_planner_ros
{

/**
 * @brief Enum to indicate the status of running the CC Affordance Planner and visualizing or executing the solved
 * trajectory
 */
enum Status
{
    PROCESSING,
    SUCCEEDED,
    FAILED,
    UNKNOWN
};

class CcAffordancePlannerRos : public rclcpp::Node
{
  public:
    // Type aliases
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveItPlanAndViz = moveit_plan_and_viz_msgs::srv::MoveItPlanAndViz;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using JointState = sensor_msgs::msg::JointState;

    /**
     * @brief Given a node name and node options, constructs a CcAffordancePlannerRos object
     *
     * @param node_name Name of the node
     * @param options Options for the node, for instance, declare parameter from overrides
     */
    explicit CcAffordancePlannerRos(const std::string &node_name, const rclcpp::NodeOptions &options);

    /**
     * @brief Given configuration settings for the CC Affordance planner and a description of the task, plan and
     * visualizes the robot joint trajectory and executes it on the robot. The planning is done from the current
     * state of the robot unless optionally a robot start configuration is passed. Optionally one can also pass a
     * pointer to get the status of this function.
     *
     * @param planner_config `cc_affordance_planner::PlannerConfig` containing the settings for the Cc Affordance
     * planner:
     * - `trajectory_density`: Specifies the density of the trajectory as the number of points.
     *   For example, an affordance goal of 0.5 rad could have 5 points, each with a step of 0.1 rad.
     * - `accuracy`: Defines the threshold for the affordance goal.
     *   For instance, for a 5 rad goal with 10% accuracy, set this parameter to 0.1 to achieve an affordance goal of 5
     * ± 0.5.
     * - `ik_max_itr`: (Advanced) Specifies the maximum iterations for the closed-chain inverse kinematics solver.
     * Default is 200.
     * - `update_method`: (Advanced) Specifies the update method to use. Possible values are `INVERSE`, `TRANSPOSE`, and
     * `BEST`. Default is `BEST`.
     * - `closure_error_threshold_ang`: (Advanced) Specifies the angular error threshold for closed-chain closure.
     * Default is 1e-4.
     * - `closure_error_threshold_lin`: (Advanced) Specifies the linear error threshold for closed-chain closure.
     * Default is 1e-5.
     *
     * @param taskDescription `cc_affordance_planner::TaskDescription` describing the task with fields:
     * - `motion_type`: `cc_affordance_planner::MotionType` describing the motion type to consider. The options are
     *   cc_affordance_planner::APPROACH or cc_affordance_planner::AFFORDANCE. AFFORDANCE is default.
     * - `affordance_info`: `affordance_util::ScrewInfo` describing the affordance with type, and
     * 	 axis/location or screw as mandatory fields.
     * - `nof_secondary_joints`: Specifies the number of secondary joints.
     *   - For affordance motion:
     *     - 1: Affordance control only.
     *     - 2: Control of affordance along with EE orientation about one axis (x, y, or z whichever is first in the
     * 		vir_screw_order specified).
     *     - 3: Control of affordance along with EE orientation about two axes (the first two specified in
     * 		vir_screw_order).
     *     - 4: Affordance and full EE orientation control.
     *   - For approach motion:
     *     - Minimum 2: Controls approach motion in the context of the affordance.
     *     - 3: Adds gripper orientation control about the next axis as specified in vir_screw_order.
     *     - 4: Adds gripper orientation control about the next two axes as specified in vir_screw_order.
     *     - 5: Adds full EE orientation.
     * - `secondary_joint_goals`: Eigen::VectorXd with desired goals for secondary joints. The size of
     *   secondary_joint_goals must match `nof_secondary_joints`. The end element of secondary_joint_goals is always
     *   affordance. For affordance motion, the EE orientation goals are inserted as needed and in the order specified
     *   in vir_screw_order. For approach motion, with nof_secondary_joints = 2, secondary_joint_goals should contain
     *   (approach_goal, affordance_goal). The EE orientation goals are inserted before the approach_goal as needed. Set
     *   approach_goal=0 for all approach motion cases as it is computed by the planner.
     * - `grasp_pose`: Eigen::MatrixXd containing the grasp pose's homogenous transformation matrix (only for APPROACH
     *   motion).
     * - `vir_screw_order`: affordance_util::VirtualScrewOrder describing the order of the joints in the virtual
     *   spherical joint of the closed-chain model. This joint describes the orientation freedom of the gripper. Default
     *   value is affordance_util::VirtualScrewOrder::XYZ.
     *
     * @param status (Optional) cc_affordance_planner_ros::Status indicating planning and execution status
     * @param robotStartConfig (Optional) Eigen::VectorXd containing the start configuration of the robot. If not
     * specified, the planning is done from the current state of the robot.
     *
     * @return bool indicating success
     */
    bool run_cc_affordance_planner(
        const cc_affordance_planner::PlannerConfig &planner_config,
        const cc_affordance_planner::TaskDescription &taskDescription,
        const std::shared_ptr<Status> status =
            std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN),
        const Eigen::VectorXd &robotStartConfig = Eigen::VectorXd());

    /**
     * @brief Given vectors of configuration settings for the CC Affordance planner along with the corresponding task
     * descriptions, plan and visualizes the robot joint trajectory that solves for all specified tasks and executes it
     * on the robot. The planning is done from the current state of the robot unless optionally a robot start
     * configuration is passed. Optionally one can also pass a pointer to get the status of this function.
     *
     * @param planner_configs `std::vector<cc_affordance_planner::PlannerConfig>` containing a series of settings for
     * the Cc Affordance planner that correspond to various tasks. Each struct has the following fields:
     * - `trajectory_density`: Specifies the density of the trajectory as the number of points.
     *   For example, an affordance goal of 0.5 rad could have 5 points, each with a step of 0.1 rad.
     * - `accuracy`: Defines the threshold for the affordance goal.
     *   For instance, for a 5 rad goal with 10% accuracy, set this parameter to 0.1 to achieve an affordance goal of 5
     * ± 0.5.
     * - `ik_max_itr`: (Advanced) Specifies the maximum iterations for the closed-chain inverse kinematics solver.
     * Default is 200.
     * - `update_method`: (Advanced) Specifies the update method to use. Possible values are `INVERSE`, `TRANSPOSE`, and
     * `BEST`. Default is `BEST`.
     * - `closure_error_threshold_ang`: (Advanced) Specifies the angular error threshold for closed-chain closure.
     * Default is 1e-4.
     * - `closure_error_threshold_lin`: (Advanced) Specifies the linear error threshold for closed-chain closure.
     * Default is 1e-5.
     *
     * @param task_descriptions `std::vector<cc_affordance_planner::TaskDescription>` describing a series of tasks. Each
     * struct has the following fields:
     * - `motion_type`: `cc_affordance_planner::MotionType` describing the motion type to consider. The options are
     *   cc_affordance_planner::APPROACH or cc_affordance_planner::AFFORDANCE. AFFORDANCE is default.
     * - `affordance_info`: `affordance_util::ScrewInfo` describing the affordance with type, and
     * 	 axis/location or screw as mandatory fields.
     * - `nof_secondary_joints`: Specifies the number of secondary joints.
     *   - For affordance motion:
     *     - 1: Affordance control only.
     *     - 2: Control of affordance along with EE orientation about one axis (x, y, or z whichever is first in the
     * 		vir_screw_order specified).
     *     - 3: Control of affordance along with EE orientation about two axes (the first two specified in
     * 		vir_screw_order).
     *     - 4: Affordance and full EE orientation control.
     *   - For approach motion:
     *     - Minimum 2: Controls approach motion in the context of the affordance.
     *     - 3: Adds gripper orientation control about the next axis as specified in vir_screw_order.
     *     - 4: Adds gripper orientation control about the next two axes as specified in vir_screw_order.
     *     - 5: Adds full EE orientation.
     * - `secondary_joint_goals`: Eigen::VectorXd with desired goals for secondary joints. The size of
     *   secondary_joint_goals must match `nof_secondary_joints`. The end element of secondary_joint_goals is always
     *   affordance. For affordance motion, the EE orientation goals are inserted as needed and in the order specified
     *   in vir_screw_order. For approach motion, with nof_secondary_joints = 2, secondary_joint_goals should contain
     *   (approach_goal, affordance_goal). The EE orientation goals are inserted before the approach_goal as needed. Set
     *   approach_goal=0 for all approach motion cases as it is computed by the planner.
     * - `grasp_pose`: Eigen::MatrixXd containing the grasp pose's homogenous transformation matrix (only for APPROACH
     *   motion).
     * - `vir_screw_order`: affordance_util::VirtualScrewOrder describing the order of the joints in the virtual
     *   spherical joint of the closed-chain model. This joint describes the orientation freedom of the gripper. Default
     *   value is affordance_util::VirtualScrewOrder::XYZ.
     *
     * @param status (Optional) cc_affordance_planner_ros::Status indicating planning and execution status
     * @param robotStartConfig (Optional) Eigen::VectorXd containing the start configuration of the robot. If not
     * specified, the planning is done from the current state of the robot.
     *
     * @return bool indicating success
     */
    bool run_cc_affordance_planner(
        const std::vector<cc_affordance_planner::PlannerConfig> &planner_configs,
        const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions,
        const std::shared_ptr<Status> status =
            std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN),
        const Eigen::VectorXd &robotStartConfig = Eigen::VectorXd());

  private:
    std::shared_ptr<Status> status_{nullptr};
    rclcpp::Logger node_logger_;       // logger associated with the node
    std::string plan_and_viz_ss_name_; // name of the planning visualization server
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        robot_traj_execution_client_; // action client to execute trajectory on the robot
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        gripper_traj_execution_client_; // action client to execute trajectory on the robot
    rclcpp::Client<MoveItPlanAndViz>::SharedPtr plan_and_viz_client_; // service client to visualize joint trajectory
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;    // joint states subscriber
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                      // buffer to lookup tf data
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // Robot ROS setup data
    std::string robot_traj_execution_as_name_;   // trajectory execution action server name
    std::string gripper_traj_execution_as_name_; // trajectory execution action server name
    std::string planning_group_;                 // name of the MoveIt planning group for visualization purposes
    std::string robot_description_parameter_;    // name of the robot description parameter
    std::string rviz_fixed_frame_; // name of the fixed frame in Rviz in reference to which EE trajectory is plotted for
                                   // visualization

    // Robot data
    Eigen::MatrixXd robot_slist_;                  // screw list for the robot
    std::vector<std::string> robot_joint_names_;   // name of the joints
    std::vector<std::string> gripper_joint_names_; // name of the joints
    Eigen::Matrix<double, 4, 4> M_;                // HTM representing the center of the robot palm
    std::string ref_frame_;                        // reference frame for the screw representations
    std::string tool_frame_;                       // name of the frame that representats the center of the palm

    affordance_util_ros::JointTrajPoint robot_joint_states_;   // processed and ordered joint states data
    affordance_util_ros::JointTrajPoint gripper_joint_states_; // processed and ordered joint states data

    /**
     * @brief Given the robot name used in creating the cca_<robot> package, returns full path to the yaml file
     * containing cc affordance robot description
     *
     * @param robot_name Name of the robot used in creating the cca_<robot> package. For instance, if the package is
     * called cca_spot, the robot name is spot
     *
     * @return Full path to the yaml file containing cc affordance robot description
     */
    static std::string get_cc_affordance_robot_description_(const std::string &robot_name);

    /**
     * @brief Callback function for the joint_states subscriber. Sets the private variable, joint_states_ which contains
     * processed and ordered joint states data
     *
     * @param msg Shared pointer to the joint states message
     */
    void joint_states_cb_(const JointState::SharedPtr msg);

    /**
     * @brief Returns the joint states of the robot. Meant to use when the robot is at the start pose of the affordance
     *
     * @return
     */
    std::pair<Eigen::VectorXd, Eigen::VectorXd> get_aff_start_joint_states_();

    /**
     * @brief Given a joint trajectory and screw axis, visualizes the trajectory and executes it on the robot
     *
     * @param trajectory A bare joint trajectory as a vector of joint states
     * @param w_aff Affordance screw axis
     * @param q_aff Affordace location
     *
     * @return True if trajectory was successfully visualized and executed. false otherwise.
     */
    bool visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory, const Eigen::VectorXd &w_aff,
                                           const Eigen::VectorXd &q_aff, const bool includes_gripper_trajectory);
    /**
     * @brief Given a joint trajectory , executes it on the robot
     *
     * @param trajectory A bare joint trajectory as a vector of joint states
     *
     * @return True if trajectory was successfully visualized and executed. false otherwise.
     */
    bool execute_trajectory_(rclcpp_action::Client<FollowJointTrajectory>::SharedPtr &traj_execution_client,
                             rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options,
                             const std::string &traj_execution_as_name, const std::vector<std::string> &joint_names,
                             const std::vector<Eigen::VectorXd> &trajectory);

    /**
     * @brief Callback to process the result from the traj_execution_as_ action server
     *
     * @param result Result from a follow_joint_trajectory action server
     */
    void robot_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    /**
     * @brief Callback to process the goal response from the traj_execution_as_ action server
     *
     * @param goal_handle A shared pointer to a follow_joint_trajectory action server goal handle
     */
    void robot_traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);
    /**
     * @brief Callback to process the result from the traj_execution_as_ action server
     *
     * @param result Result from a follow_joint_trajectory action server
     */
    void gripper_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    /**
     * @brief Callback to process the goal response from the traj_execution_as_ action server
     *
     * @param goal_handle A shared pointer to a follow_joint_trajectory action server goal handle
     */
    void gripper_traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);
};
} // namespace cc_affordance_planner_ros

#endif // CC_AFFORDANCE_PLANNER_ROS
