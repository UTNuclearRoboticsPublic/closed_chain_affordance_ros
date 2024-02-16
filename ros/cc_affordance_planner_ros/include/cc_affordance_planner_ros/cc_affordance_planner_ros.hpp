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
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cmath>
#include <cstdlib>
#include <future>
#include <moveit_plan_and_viz/srv/move_it_plan_and_viz.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

class CcAffordancePlannerRos : public rclcpp::Node
{
  public:
    // Namespaces
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveItPlanAndViz = moveit_plan_and_viz::srv::MoveItPlanAndViz;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using JointState = sensor_msgs::msg::JointState;

    // Constructor
    explicit CcAffordancePlannerRos(const rclcpp::NodeOptions &options);

    void run_cc_affordance_planner(const Eigen::Vector3d &w_aff, const Eigen::Vector3d &q_aff, const double &aff_goal,
                                   const double &aff_step = 0.3, const int &gripper_control_par_tau = 1,
                                   const double &accuracy = 10.0 / 100.0);
    void run_cc_affordance_planner(const Eigen::Vector3d &w_aff, const std::string &apriltag_frame_name,
                                   const double &aff_goal, const double &aff_step = 0.3,
                                   const int &gripper_control_par_tau = 1, const double &accuracy = 10.0 / 100.0);

  private:
    rclcpp::Logger node_logger_; // logger associated with the node
    std::string plan_and_viz_ss_name_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr traj_execution_client_;
    rclcpp::Client<MoveItPlanAndViz>::SharedPtr plan_and_viz_client_; // Service client to visualize joint trajectory
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;    // Joint states subscriber
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                      // Buffer to lookup tf data

    // Robot ROS setup data
    std::string traj_execution_as_name_;
    std::string planning_group_;
    std::string robot_description_parameter_;
    std::string rviz_fixed_frame_;

    // Robot data
    Eigen::MatrixXd robot_slist_;
    std::vector<std::string> joint_names_;
    Eigen::Matrix<double, 4, 4> M_;
    std::string ref_frame_;
    std::string tool_frame_;

    AffordanceUtilROS::JointTrajPoint joint_states_; // Processed and ordered joint states data

    // Returns full path to the yaml file containing cc affordance robot description
    static std::string get_cc_affordance_robot_description_(const std::string &robot_name);

    // Callback function for the joint_states subscriber
    void joint_states_cb_(const JointState::SharedPtr msg);

    // Function to read robot joint states at the start of the affordance
    Eigen::VectorXd get_aff_start_joint_states_();

    // Function to visualize and execute planned trajectory
    void visualize_and_execute_trajectory_(std::vector<Eigen::VectorXd> trajectory);

    // Callback to process traj_execution_as feedback
    void traj_execution_feedback_callback_(GoalHandleFollowJointTrajectory::SharedPtr,
                                           const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);

    // Callback to process traj_execution_as result
    void traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    // Callback to process traj_exection_as goal response
    void traj_execution_goal_response_callback_(std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> future);
};

#endif // CC_AFFORDANCE_PLANNER_ROS
