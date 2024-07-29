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
#include <cmath>
#include <cstdlib>
#include <future>
#include <moveit_plan_and_viz/srv/move_it_plan_and_viz.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

namespace cc_affordance_planner_ros
{

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
    // Namespaces
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveItPlanAndViz = moveit_plan_and_viz::srv::MoveItPlanAndViz;
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
     * @brief Generates, visualizes, and executes joint trajectories using the Closed-Chain Affordance model based on
     *        given planner configuration, affordance screw information, secondary joint goals, and gripper control
     *        parameters.
     *
     * @param plannerConfig cc_affordance_planner::PlannerConfig containing CC Affordance Planner configuration
     *        settings:
     *        - Use the parameter `aff_step` to specify the trajectory density with an affordance step.
     *          For example, an affordance goal of 0.5 rad could have an affordance step of 0.1 rad,
     *          resulting in a trajectory with 5 points, where the intermediate affordance goals are 0.1, 0.2, 0.3, 0.4,
     * 		and 0.5 rad.
     *        - The `accuracy` parameter represents the threshold for the affordance goal (and step). For instance,
     *          if 10% accuracy is desired for a 1 rad goal, set accuracy to 0.1. This will produce joint solutions
     *          that result in an affordance goal of 1 ± 0.1 rad.
     *        - Advanced users can utilize two additional parameters:
     *          - `closure_error_threshold`: Specify the error threshold for the closed-chain closure error.
     *          - `max_itr`: Specify the maximum iterations for the closed-chain inverse kinematics solver.
     *
     * @param aff affordance_util::ScrewInfo containing information about the affordance. The `aff_type` and `axis`
     * 	      fields are mandatory. Possible values for `aff_type` include "translation", "rotation", and "screw".
     * 	      Specify `location_frame` if using AprilTag.
     *
     * @param sec_goal Eigen::VectorXd containing secondary joint angle goals including EE orientation and affordance,
     *        with the affordance goal being the end element.
     *
     * @param gripper_control_par_tau A numeric parameter indicating the length of the secondary joint vector:
     *        - A value of 1 implies only affordance control.
     *        - A value of 2 represents affordance control along with controlling the gripper orientation about the next
     * 		adjacent virtual gripper axis (x, y, or z).
     *        - A value of 3 involves controlling affordance along with the EE orientation about the next two virtual
     * 		gripper axes.
     *        - A value of 4 refers to affordance control along with all aspects of EE orientation.
     *
     * @param vir_screw_order std::string indicating the order for the virtual EE screws. Possible values are "xyz",
     * "yzx" and "zxy". Default is "xyz"
     *
     * @param status cc_affordance_planner_ros::Status indicating planning and executation status
     *
     * @param robot_start_config Eigen::VectorXd containing the start configuration of the robot. This is internally
     * read via the robot joint_states topic. Here for testing purposes only, when planning without the joint_states
     * topic.
     *
     * @return bool indicating success
     */

    bool run_cc_affordance_planner(
        const cc_affordance_planner::PlannerConfig &plannerConfig, affordance_util::ScrewInfo &aff,
        const Eigen::VectorXd &sec_goal, const size_t &gripper_control_par = 1,
        const std::string &vir_screw_order = "xyz",
        const std::shared_ptr<Status> status =
            std::make_shared<cc_affordance_planner_ros::Status>(cc_affordance_planner_ros::Status::UNKNOWN),
        Eigen::VectorXd robot_start_config = Eigen::VectorXd());

  private:
    std::shared_ptr<Status> status_{nullptr};
    rclcpp::Logger node_logger_;       // logger associated with the node
    std::string plan_and_viz_ss_name_; // name of the planning visualization server
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr
        traj_execution_client_; // action client to execute trajectory on the robot
    rclcpp::Client<MoveItPlanAndViz>::SharedPtr plan_and_viz_client_; // service client to visualize joint trajectory
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;    // joint states subscriber
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                      // buffer to lookup tf data
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    // Robot ROS setup data
    std::string traj_execution_as_name_;      // trajectory execution action server name
    std::string planning_group_;              // name of the MoveIt planning group for visualization purposes
    std::string robot_description_parameter_; // name of the robot description parameter
    std::string rviz_fixed_frame_; // name of the fixed frame in Rviz in reference to which EE trajectory is plotted for
                                   // visualization

    // Robot data
    Eigen::MatrixXd robot_slist_;          // screw list for the robot
    std::vector<std::string> joint_names_; // name of the joints
    Eigen::Matrix<double, 4, 4> M_;        // HTM representing the center of the robot palm
    std::string ref_frame_;                // reference frame for the screw representations
    std::string tool_frame_;               // name of the frame that representats the center of the palm

    affordance_util_ros::JointTrajPoint joint_states_; // processed and ordered joint states data

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
    Eigen::VectorXd get_aff_start_joint_states_();

    /**
     * @brief
     *
     * @param trajectory A bare joint trajectory as a vector of joint position points
     * @param w_aff Affordance screw axis
     * @param q_aff Affordace location
     *
     * @return True if trajectory was successfully visualized and executed. false otherwise.
     */
    bool visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory,
                                           const Eigen::VectorXd &robot_start_config, const Eigen::VectorXd &w_aff,
                                           const Eigen::VectorXd &q_aff);
    bool execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory, const Eigen::VectorXd &robot_start_config);
    /**
     * @brief Callback function to process the feedback from the traj_execution_as_ action server
     *
     * @param GoalHandleFollowJointTrajectory::SharedPtr Goal handle to a follow_joint_trajectory action server
     * @param feedback Feedback message from a follow_joint_trajectory action server
     */
    void traj_execution_feedback_callback_(GoalHandleFollowJointTrajectory::SharedPtr,
                                           const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback);

    /**
     * @brief Callback to process the result from the traj_execution_as_ action server
     *
     * @param result Result from a follow_joint_trajectory action server
     */
    void traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result);

    /**
     * @brief Callback to process the goal response from the traj_execution_as_ action server
     *
     * @param goal_handle A shared pointer to a follow_joint_trajectory action server goal handle
     */
    void traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle);
};
} // namespace cc_affordance_planner_ros

#endif // CC_AFFORDANCE_PLANNER_ROS
