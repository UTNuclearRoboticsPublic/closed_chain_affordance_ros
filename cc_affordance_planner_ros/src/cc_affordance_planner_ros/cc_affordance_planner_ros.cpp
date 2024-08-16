#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

namespace cc_affordance_planner_ros
{
CcAffordancePlannerRos::CcAffordancePlannerRos(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options), // Use new class name
      node_logger_(this->get_logger()),
      plan_and_viz_ss_name_("/moveit_plan_and_viz_server")
{
    // Extract CC Affordance parameters regarding ROS-related setup
    traj_execution_as_name_ = this->get_parameter("cca_robot_as").as_string();
    robot_description_parameter_ = this->get_parameter("cca_robot_description_parameter").as_string();
    planning_group_ = this->get_parameter("cca_planning_group").as_string();
    rviz_fixed_frame_ = this->get_parameter("rviz_fixed_frame").as_string();
    const std::string joint_states_topic = this->get_parameter("cca_joint_states_topic").as_string();
    const std::string robot_name = this->get_parameter("cca_robot").as_string();

    // Extract robot config info
    const std::string robot_config_file_path = CcAffordancePlannerRos::get_cc_affordance_robot_description_(robot_name);
    const affordance_util::RobotConfig &robotConfig = affordance_util::robot_builder(robot_config_file_path);
    robot_slist_ = robotConfig.Slist;
    M_ = robotConfig.M;
    ref_frame_ = robotConfig.ref_frame_name;
    tool_frame_ = robotConfig.tool_name;
    joint_names_ = robotConfig.joint_names;

    // Initialize clients and subscribers
    traj_execution_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, traj_execution_as_name_);
    plan_and_viz_client_ = this->create_client<MoveItPlanAndViz>(plan_and_viz_ss_name_);
    joint_states_sub_ = this->create_subscription<JointState>(
        joint_states_topic, 1000, std::bind(&CcAffordancePlannerRos::joint_states_cb_, this, std::placeholders::_1));

    // Construct buffer to lookup affordance location from apriltag using tf data
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool CcAffordancePlannerRos::run_cc_affordance_planner(const cc_affordance_planner::PlannerConfig &planner_config,
                                                       const cc_affordance_planner::TaskDescription &TaskDescription,
                                                       const std::shared_ptr<Status> status,
                                                       const Eigen::VectorXd &robotStartConfig)
{
    status_ = status;
    *status_ = Status::PROCESSING;

    // Copy task description and robot start config in case we need to make modifications before calling the planner
    cc_affordance_planner::TaskDescription task_description = TaskDescription;
    Eigen::VectorXd robot_start_config = robotStartConfig;

    // If tag frame is specified then, we lookup affordance location from tag
    if (!task_description.affordance_info.location_frame.empty())
    {

        RCLCPP_INFO_STREAM(node_logger_, "Ready to read affordance location from apriltag? y or Y for yes.");
        std::string conf;
        std::cin >> conf;
        if (conf != "y" && conf != "Y")
        {
            throw std::runtime_error("You indicated you are not ready to read affordance location");
        }
        const Eigen::Isometry3d aff_htm =
            affordance_util_ros::get_htm(ref_frame_, task_description.affordance_info.location_frame, *tf_buffer_);
        task_description.affordance_info.location = aff_htm.translation();
    }

    // Get joint states at the start configuration of the affordance
    if (robot_start_config.size() == 0) // Non-zero if testing or planning without the joint_states topic
    {
        robot_start_config = get_aff_start_joint_states_();
    }

    // Fill out robot description
    affordance_util::RobotDescription robot_description;
    robot_description.slist = robot_slist_;
    robot_description.M = M_;
    robot_description.joint_states = robot_start_config;

    /* // Compose cc model and affordance goal */
    /* RCLCPP_INFO(this->get_logger(), "DEBUG FLAG"); */
    /* RCLCPP_INFO_STREAM(node_logger_, "Here is the gripper_control_par to the planner\n" << gripper_control_par); */
    /* RCLCPP_INFO_STREAM(node_logger_, "Here is the robot start config\n" << robot_start_config); */
    /* RCLCPP_INFO_STREAM(node_logger_, "Here is the vir screw order\n" << vir_screw_order); */
    /* RCLCPP_INFO_STREAM(node_logger_, "Here is the approach_end_pose\n" << approach_end_pose); */

    /* affordance_util::CcModel cc_model = affordance_util::compose_cc_model_slist( */
    /*     robot_slist_, robot_start_config, M_, aff, vir_screw_order, approach_end_pose); */
    /* plannerConfig.aff_step = cc_model.approach_limit / 10.0; */
    /* RCLCPP_INFO_STREAM(node_logger_, "Here is the cc_slist to the planner\n" << cc_model.slist); */
    /* sec_goal.head(1)(0) = cc_model.approach_limit; */
    /* RCLCPP_INFO_STREAM(node_logger_, "Here is the approach limit to the planner\n" << cc_model.approach_limit); */
    /* RCLCPP_INFO_STREAM(node_logger_, "Here is the sec_goal to the planner\n" << sec_goal); */
    /* /1* cc_model.slist.col(6) = -cc_model.slist.col(6); *1/ */

    // Run the planner
    cc_affordance_planner::PlannerResult plannerResult =
        cc_affordance_planner::generate_joint_trajectory(planner_config, robot_description, task_description);

    // Print planner result
    std::vector<Eigen::VectorXd> solution = plannerResult.joint_trajectory;
    if (plannerResult.success)
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with update trail '"
                                             << plannerResult.update_trail << "' and the planning took "
                                             << plannerResult.planning_time.count() << " microseconds.");
        switch (plannerResult.trajectory_description)
        {

        case cc_affordance_planner::TrajectoryDescription::FULL:
            RCLCPP_INFO(node_logger_, "Trajectory description: FULL.");
            break;
        case cc_affordance_planner::TrajectoryDescription::PARTIAL:
            RCLCPP_WARN(node_logger_, "Trajectory description: PARTIAL. Execute with caution.");
            break;
        default:
            RCLCPP_ERROR(node_logger_, "Trajectory description: UNSET.");
            break;
        }

        /* RCLCPP_INFO_STREAM(node_logger_, "Here is the solution\n"); */
        /* for (const auto &point : solution) */
        /* { */
        /*     RCLCPP_INFO_STREAM(node_logger_, point.transpose()); */
        /* } */
        /* /1* if (plannerResult.traj_full_or_partial != "full") *1/ */
        /* /1* { *1/ */
        /* /1*     *status_ = Status::FAILED; *1/ */
        /* /1*     RCLCPP_ERROR(node_logger_, "Planner returned a partial trajectory. Visualize before executing."); *1/
         */
        /* /1*     return false; *1/ */
        /* /1* } *1/ */
    }
    else
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
        *status_ = Status::FAILED;
        return false;
    }

    // Visualize and execute trajectory
    return visualize_and_execute_trajectory_(solution, robot_start_config, task_description.affordance_info.axis,
                                             task_description.affordance_info.location);
    // Execute trajectory
    /* return execute_trajectory_(solution, robot_start_config); */
}

/* bool CcAffordancePlannerRos::run_cc_affordance_planner(const cc_affordance_planner::PlannerConfig &plannerConfig, */
/*                                                        affordance_util::ScrewInfo &aff, const Eigen::VectorXd
 * &sec_goal, */
/*                                                        const size_t &gripper_control_par, */
/*                                                        const std::string &vir_screw_order, */
/*                                                        const std::shared_ptr<Status> status, */
/*                                                        Eigen::VectorXd robot_start_config) */
/* { */
/*     status_ = status; */
/*     *status_ = Status::PROCESSING; */
/*     // If tag frame is specified then, we lookup affordance location from tag */
/*     if (!aff.location_frame.empty()) */
/*     { */

/*         RCLCPP_INFO_STREAM(node_logger_, "Ready to read affordance location from apriltag? y or Y for yes."); */
/*         std::string conf; */
/*         std::cin >> conf; */
/*         if (conf != "y" && conf != "Y") */
/*         { */
/*             throw std::runtime_error("You indicated you are not ready to read affordance location"); */
/*         } */
/*         const Eigen::Isometry3d aff_htm = affordance_util_ros::get_htm(ref_frame_, aff.location_frame, *tf_buffer_);
 */
/*         aff.location = aff_htm.translation(); */
/*     } */

/*     // Get joint states at the start configuration of the affordance */
/*     if (robot_start_config.size() == 0) // Non-zero if testing or planning without the joint_states topic */
/*     { */
/*         robot_start_config = get_aff_start_joint_states_(); */
/*     } */

/*     // Compose cc model and affordance goal */
/*     Eigen::MatrixXd cc_slist = */
/*         affordance_util::compose_cc_model_slist(robot_slist_, robot_start_config, M_, aff, vir_screw_order); */

/*     // Run the planner */
/*     cc_affordance_planner::PlannerResult plannerResult = */
/*         cc_affordance_planner::generate_joint_trajectory(plannerConfig, cc_slist, sec_goal, gripper_control_par); */

/*     // Print planner result */
/*     std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj; */
/*     if (plannerResult.success) */
/*     { */
/*         RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with " */
/*                                              << plannerResult.traj_full_or_partial << " solution, planning took " */
/*                                              << plannerResult.planning_time.count() << " microseconds, and " */
/*                                              << plannerResult.update_method << " update method was used."); */
/*         /1* if (plannerResult.traj_full_or_partial != "full") *1/ */
/*         /1* { *1/ */
/*         /1*     *status_ = Status::FAILED; *1/ */
/*         /1*     RCLCPP_ERROR(node_logger_, "Planner returned a partial trajectory. Visualize before executing."); *1/
 */
/*         /1*     return false; *1/ */
/*         /1* } *1/ */
/*     } */
/*     else */
/*     { */
/*         RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution"); */
/*         *status_ = Status::FAILED; */
/*         return false; */
/*     } */

/*     // Visualize and execute trajectory */
/*     return visualize_and_execute_trajectory_(solution, robot_start_config, aff.axis, aff.location); */
/*     // Execute trajectory */
/*     /1* return execute_trajectory_(solution, robot_start_config); *1/ */
/* } */

// Returns full path to the yaml file containing cc affordance robot description
std::string CcAffordancePlannerRos::get_cc_affordance_robot_description_(const std::string &robot_name)
{
    const std::string package_name = "cca_" + robot_name;            // get package name from the parameter server
    const std::string rel_dir = "/config/";                          // relative directory where yaml file is located
    const std::string filename = package_name + "_description.yaml"; // yaml file name
    return affordance_util_ros::get_filepath_inside_pkg(package_name, rel_dir, filename);
}

// Callback function for the joint_states subscriber
void CcAffordancePlannerRos::joint_states_cb_(const JointState::SharedPtr msg)
{
    joint_states_ = affordance_util_ros::get_ordered_joint_states(
        msg,
        joint_names_); // Takes care of filtering and ordering the joint_states
}

// Function to read robot joint states at the start of the affordance
Eigen::VectorXd CcAffordancePlannerRos::get_aff_start_joint_states_()
{
    // Set Eigen::VectorXd size
    joint_states_.positions.conservativeResize(joint_names_.size());

    if (joint_states_.positions.isZero())
    {

        throw std::runtime_error("Could not read joint states.");
    }

    return joint_states_.positions;
}

// Function to visualize and execute planned trajectory
bool CcAffordancePlannerRos::visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory,
                                                               const Eigen::VectorXd &robot_start_config,
                                                               const Eigen::VectorXd &w_aff,
                                                               const Eigen::VectorXd &q_aff)
{

    // Visualize trajectory in RVIZ
    // Convert the solution trajectory to ROS message type
    const double traj_time_step = 0.3;
    const control_msgs::action::FollowJointTrajectory_Goal goal =
        affordance_util_ros::follow_joint_trajectory_msg_builder(
            trajectory, Eigen::VectorXd::Zero(6), joint_names_,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains the whole closed-chain trajectory
    std::cout << "Here is the final trajectory:" << std::endl;
    for (const auto &point : goal.trajectory.points)
    {

        for (int i = 0; i < point.positions.size(); i++)
        {
            std::cout << point.positions[i] << " ";
        }
        std::cout << std::endl;
    }

    // Fill out service request
    auto plan_and_viz_serv_req = std::make_shared<MoveItPlanAndViz::Request>();
    plan_and_viz_serv_req->joint_traj = goal.trajectory;
    plan_and_viz_serv_req->aff_screw_axis = {w_aff[0], w_aff[1], w_aff[2]};
    plan_and_viz_serv_req->aff_location = {q_aff[0], q_aff[1], q_aff[2]};
    plan_and_viz_serv_req->ref_frame = ref_frame_;
    plan_and_viz_serv_req->tool_frame = tool_frame_;
    plan_and_viz_serv_req->planning_group = planning_group_;
    plan_and_viz_serv_req->robot_description = robot_description_parameter_;
    plan_and_viz_serv_req->rviz_fixed_frame = rviz_fixed_frame_;

    // Call service to visualize
    while (!plan_and_viz_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node_logger_, "Interrupted while waiting for %s service. Exiting.",
                         plan_and_viz_ss_name_.c_str());
            return false;
        }
        RCLCPP_INFO(node_logger_, " %s service not available, waiting again...", plan_and_viz_ss_name_.c_str());
    }

    auto result_future = plan_and_viz_client_->async_send_request(plan_and_viz_serv_req);
    RCLCPP_INFO(node_logger_, "Waiting on %s service to complete", plan_and_viz_ss_name_.c_str());
    auto response = result_future.get(); // blocks until response is received

    if (response->success)
    {
        RCLCPP_INFO(node_logger_, " %s service succeeded", plan_and_viz_ss_name_.c_str());

        // Execute trajectory on the real robot
        RCLCPP_INFO_STREAM(node_logger_, "Ready to execute the trajectory? y to confirm");
        std::string execution_conf;
        std::cin >> execution_conf;

        if (execution_conf != "y" && execution_conf != "Y")
        {
            RCLCPP_INFO_STREAM(node_logger_, "Trajectory execution was canceled");
            return false;
        }

        // Send the goal to follow_joint_trajectory action server for execution
        if (!this->traj_execution_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_logger_, " %s action server not available after waiting",
                         traj_execution_as_name_.c_str());
            return false;
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
        return true;
    }
    else
    {
        RCLCPP_ERROR(node_logger_, "%s service call failed", plan_and_viz_ss_name_.c_str());
        return false;
    }
}

// Function to execute planned trajectory
bool CcAffordancePlannerRos::execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory,
                                                 const Eigen::VectorXd &robot_start_config)
{

    // Visualize trajectory in RVIZ
    // Convert the solution trajectory to ROS message type
    const double traj_time_step = 0.3;
    const control_msgs::action::FollowJointTrajectory_Goal goal =
        affordance_util_ros::follow_joint_trajectory_msg_builder(
            trajectory, robot_start_config, joint_names_,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains qs data too

    // Send the goal to follow_joint_trajectory action server for execution
    if (!this->traj_execution_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(node_logger_, " %s action server not available after waiting", traj_execution_as_name_.c_str());
        *status_ = Status::FAILED;
        return false;
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
    return true;
}
// Callback to process traj_execution_as feedback
/* void traj_execution_feedback_callback_(GoalHandleFollowJointTrajectory::SharedPtr, */
/*                                        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) */
/* { */
/*     // Empty for now */
/* } */

// Callback to process traj_execution_as result
void CcAffordancePlannerRos::traj_execution_result_callback_(
    const GoalHandleFollowJointTrajectory::WrappedResult &result)
{
    *status_ = Status::UNKNOWN;
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
    *status_ = Status::SUCCEEDED;
    RCLCPP_INFO(node_logger_, "%s action server call concluded", traj_execution_as_name_.c_str());
}

// Callback to process traj_exection_as goal response
void CcAffordancePlannerRos::traj_execution_goal_response_callback_(
    const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
{
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
} // namespace cc_affordance_planner_ros
