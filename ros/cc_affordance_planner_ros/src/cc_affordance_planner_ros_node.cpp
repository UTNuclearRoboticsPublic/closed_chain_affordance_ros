///////////////////////////////////////////////////////////////////////////////
//      Title     : cc_affordance_planner_ros_node.hpp
//      Project   : cc_affordance_planner_ros_
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

// Description: Primary customization points are within the main function and are clearly highlighted with /*---*/ for
// user convenience. The remainder of the code is encapsulated to cater to advanced users, reducing the likelihood of
// unintended modifications.

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
    explicit CcAffordancePlannerRos(const rclcpp::NodeOptions &options)
        : Node("cc_affordance_planner_ros", options), // Use new class name
          node_logger_(this->get_logger()),
          plan_and_viz_ss_name_("/moveit_plan_and_viz_server"),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
    }

    void lookup_robot_ros_setup_info()
    {
        executor_->add_node(this->shared_from_this());
        std::thread([this]() { executor_->spin(); }).detach();
        // Lookup robot_description and planning group name from the global parameter server
        RCLCPP_INFO_STREAM(node_logger_, "Waiting on global parameter server");
        global_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "/global_parameter_server");
        auto parameters =
            global_parameters_client_->get_parameters({"cca_robot", "cca_robot_as", "cca_joint_states_topic"});
        robot_name_ = parameters.get()[0].value_to_string();
        traj_execution_as_name_ = parameters.get()[1].value_to_string();
        joint_states_topic_ = parameters.get()[2].value_to_string();
        RCLCPP_INFO_STREAM(node_logger_, "Found global parameter server");
        if (traj_execution_as_name_ == "not set" || joint_states_topic_ == "not set" || robot_name_ == "not set")
        {
            RCLCPP_ERROR(node_logger_, "Some parameters are not set. Ensure all parameters are set in "
                                       "cc_affordance_<robot>_ros_setup.yaml");
        }
        traj_execution_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, traj_execution_as_name_);
        joint_states_sub_ = this->create_subscription<JointState>(
            joint_states_topic_, 1000,
            std::bind(&CcAffordancePlannerRos::joint_states_cb_, this, std::placeholders::_1));
        // Initialize clients and subscribers
        plan_and_viz_client_ = this->create_client<MoveItPlanAndViz>(plan_and_viz_ss_name_);

        // Extract robot config info
        const std::string robot_config_file_path = get_cc_affordance_robot_description_();
        const AffordanceUtil::RobotConfig &robotConfig = AffordanceUtil::robot_builder(robot_config_file_path);
        robot_slist_ = robotConfig.Slist;
        M_ = robotConfig.M;
        ref_frame_ = robotConfig.ref_frame_name;
        tool_frame_ = robotConfig.tool_name;
        joint_names_ = robotConfig.joint_names;
    }

    void run_cc_affordance_planner(const Eigen::VectorXd &aff_screw, const double &aff_goal,
                                   const double &aff_step = 0.3, const int &gripper_control_par_tau = 1,
                                   const double &accuracy = 10.0 / 100.0)
    {
        // Get joint states at the start configuration of the affordance
        Eigen::VectorXd robot_thetalist = get_aff_start_joint_states_();

        // Compose cc model and affordance goal
        Eigen::MatrixXd cc_slist = AffordanceUtil::compose_cc_model_slist(robot_slist_, robot_thetalist, M_, aff_screw);

        // Construct the CcAffordancePlanner object
        CcAffordancePlanner ccAffordancePlanner;

        // Set planner parameters
        auto sign_of = [](double x) {
            return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
        }; // Helper lambda to check the sign of affordance goal

        ccAffordancePlanner.p_aff_step_deltatheta_a = sign_of(aff_goal) * abs(aff_step);
        ccAffordancePlanner.p_task_err_threshold_eps_s = accuracy * aff_step;

        PlannerResult plannerResult =
            ccAffordancePlanner.affordance_stepper(cc_slist, aff_goal, gripper_control_par_tau);

        // Print planner result
        std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
        if (plannerResult.success)
        {
            RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with " << plannerResult.traj_full_or_partial
                                                                       << " solution, and planning took "
                                                                       << plannerResult.planning_time.count()
                                                                       << " microseconds");
        }
        else
        {
            RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
        }

        // Visualize and execute trajectory
        visualize_and_execute_trajectory_(solution);
    }

  private:
    rclcpp::Logger node_logger_; // logger associated with the node
    std::string traj_execution_as_name_;
    std::string plan_and_viz_ss_name_;
    std::string joint_states_topic_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr traj_execution_client_;
    rclcpp::Client<MoveItPlanAndViz>::SharedPtr plan_and_viz_client_; // Service client to visualize joint trajectory
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;    // Joint states subscriber
    rclcpp::AsyncParametersClient::SharedPtr
        global_parameters_client_; // client to get parameter values from the global parameter server
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
        executor_; // executor needed for MoveIt robot state checking
    std::string robot_name_;

    // Robot data
    Eigen::MatrixXd robot_slist_;
    std::vector<std::string> joint_names_;
    Eigen::MatrixXd M_;
    std::string ref_frame_;
    std::string tool_frame_;

    AffordanceUtilROS::JointTrajPoint joint_states_; // Processed and ordered joint states data

    // Returns full path to the yaml file containing cc affordance robot description
    std::string get_cc_affordance_robot_description_()
    {
        const std::string package_name =
            "cc_affordance_" + robot_name_ + "_description"; // get package name from the parameter server
        const std::string rel_dir = "/config/";              // relative directory where yaml file is located
        const std::string filename = package_name + ".yaml"; // yaml file name
        return AffordanceUtilROS::get_filepath_inside_pkg(package_name, rel_dir, filename);
    }

    // Callback function for the joint_states subscriber
    void joint_states_cb_(const JointState::SharedPtr msg)
    {
        joint_states_ = AffordanceUtilROS::get_ordered_joint_states(
            msg,
            joint_names_); // Takes care of filtering and ordering the joint_states
    }

    // Function to read robot joint states at the start of the affordance
    Eigen::VectorXd get_aff_start_joint_states_()
    {
        // Set Eigen::VectorXd size
        joint_states_.positions.conservativeResize(joint_names_.size());

        // Capture initial configuration joint states
        RCLCPP_INFO_STREAM(node_logger_,
                           "Put the robot in the start configuration for affordance execution. Done? y for yes.");
        std::string capture_joint_states_conf;
        std::cin >> capture_joint_states_conf;

        if (capture_joint_states_conf != "y" && capture_joint_states_conf != "Y")
        {
            throw std::runtime_error("You indicated you are not ready to capture joint states");
        }

        return joint_states_.positions;
    }

    // Function to visualize and execute planned trajectory
    void visualize_and_execute_trajectory_(std::vector<Eigen::VectorXd> trajectory)
    {
        // Visualize trajectory in RVIZ
        // Convert the solution trajectory to ROS message type
        const double traj_time_step = 0.3;
        const control_msgs::action::FollowJointTrajectory_Goal goal =
            AffordanceUtilROS::follow_joint_trajectory_msg_builder(
                trajectory, joint_states_.positions, joint_names_,
                traj_time_step); // this function takes care of extracting the right
                                 // number of joint_states although solution
                                 // contains qs data too

        // Fill out service request
        auto plan_and_viz_serv_req = std::make_shared<MoveItPlanAndViz::Request>();
        plan_and_viz_serv_req->joint_traj = goal.trajectory;
        plan_and_viz_serv_req->ref_frame = ref_frame_;
        plan_and_viz_serv_req->tool_frame = tool_frame_;

        // Call service to visualize
        while (!plan_and_viz_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_logger_, "Interrupted while waiting for %s service. Exiting.",
                             plan_and_viz_ss_name_.c_str());
                return;
            }
            RCLCPP_INFO(node_logger_, plan_and_viz_ss_name_.c_str(), " service not available, waiting again...");
        }

        auto result = plan_and_viz_client_->async_send_request(plan_and_viz_serv_req);
        /* // Wait for the result. */
        /* if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == */
        /*     rclcpp::FutureReturnCode::SUCCESS) */
        /* { */
        /*     RCLCPP_INFO(node_logger_, plan_and_viz_ss_name_.c_str(), " service succeeded"); */
        /* } */
        /* else */
        /* { */
        /*     RCLCPP_ERROR(node_logger_, plan_and_viz_ss_name_.c_str(), "service call failed"); */
        /* } */

        // Execute trajectory on the real robot

        RCLCPP_INFO_STREAM(node_logger_, "Ready to execute the trajectory? y to confirm");
        std::string execution_conf;
        std::cin >> execution_conf;

        if (execution_conf != "y" && execution_conf != "Y")
        {
            RCLCPP_INFO_STREAM(node_logger_, "You canceled trajectory execution");
            return;
        }

        // Send the goal to follow_joint_trajectory action server for execution
        if (!this->traj_execution_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_logger_, " %s action server not available after waiting",
                         traj_execution_as_name_.c_str());
            rclcpp::shutdown();
        }

        RCLCPP_INFO(node_logger_, "Sending goal to %s action server", traj_execution_as_name_.c_str());

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_goal_response_callback_, this, std::placeholders::_1);
        /* send_goal_options.feedback_callback =
         * std::bind(&CcAffordancePlannerRos::traj_execution_feedback_callback_,
         */
        /*                                                 this, std::placeholders::_1, std::placeholders::_2); */
        send_goal_options.result_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_result_callback_, this, std::placeholders::_1);

        this->traj_execution_client_->async_send_goal(goal, send_goal_options);
    }

    // Callback to process traj_execution_as feedback
    /* void traj_execution_feedback_callback_(GoalHandleFollowJointTrajectory::SharedPtr, */
    /*                                        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) */
    /* { */
    /*     // Empty for now */
    /* } */

    // Callback to process traj_execution_as result
    void traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_logger_, "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_logger_, "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(node_logger_, "Unknown result code");
            return;
        }
        RCLCPP_INFO(node_logger_, traj_execution_as_name_.c_str(), "action server call concluded");
        rclcpp::shutdown();
    }

    // Callback to process traj_exection_as goal response
    void traj_execution_goal_response_callback_(std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_logger_, "Goal was rejected by %s action server", traj_execution_as_name_.c_str());
        }
        else
        {
            RCLCPP_INFO(node_logger_, "Goal accepted by %s action server, waiting for result",
                        traj_execution_as_name_.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcAffordancePlannerRos>(node_options);
    node->lookup_robot_ros_setup_info(); // lookup trajectory as name and joint states topic name from the global
                                         // parameter server

    // Construct buffer to lookup tf data for the tag
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
    tf2::Duration timeout(1s);
    tf2_ros::Buffer tf_buffer(clock, timeout, node);

    // Compose affordance screw
    /*------------------------------------------------------------*/
    const bool aff_from_tag = false;
    /*------------------------------------------------------------*/
    Eigen::Matrix<double, 6, 1> aff_screw;
    Eigen::Vector3d w_aff; // screw axis
    Eigen::Vector3d q_aff; // location vector

    if (aff_from_tag)
    {
        /*------------------------------------------------------------*/
        const std::string affordance_frame = "affordance_frame"; // Name of the tag frame
        const std::string reference_frame = "arm0_base_link";    // Maybe we can move the aff_from_tag portion inside
                                                                 // the class later since reference_frame info is
                                                                 // available there.
        w_aff << -1, 0, 0;                                       // screw axis
        /*------------------------------------------------------------*/

        // Extract Affordance frame location from TF data
        const Eigen::Isometry3d tag_htm = AffordanceUtilROS::get_htm(reference_frame, affordance_frame, tf_buffer);
        q_aff = tag_htm.translation();
        RCLCPP_INFO_STREAM(node->get_logger(), "Here is the affordance frame location. Ensure it makes sense:\n"
                                                   << q_aff);
    }
    else
    {
        /*------------------------------------------------------------*/
        w_aff << 0, 0, 1;
        q_aff << 0, 0, 0;
        /*------------------------------------------------------------*/
    }

    // Compute the 6x1 screw vector
    aff_screw = AffordanceUtil::get_screw(w_aff, q_aff); // compute affordance screw

    /*------------------------------------------------------------*/
    // Set affordance goal
    const double aff_goal = 0.5 * M_PI; // Code
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Optionally set planner parameters
    /* const double &aff_step = 0.3; */
    /* const int &gripper_control_par_tau = 1; */
    /* const double &accuracy = 10.0 / 100.0; */
    /*------------------------------------------------------------*/

    // Run the planner
    node->run_cc_affordance_planner(aff_screw, aff_goal);
    /*------------------------------------------------------------*/
    // Optionally, with planner parameters call:
    /* node->run_cc_affordance_planner(aff_screw, aff_goal, aff_step, gripper_control_par_tau, accuracy); */
    /*------------------------------------------------------------*/

    // Call the function again with new (or old) aff_screw and aff_goal to execute
    // another affordance in series
    /* node->run_cc_affordance_planner(aff_screw, aff_goal); */

    /* rclcpp::spin(node); // Keep node alive */
    while (rclcpp::ok())
    {
    }

    return 0;
}
