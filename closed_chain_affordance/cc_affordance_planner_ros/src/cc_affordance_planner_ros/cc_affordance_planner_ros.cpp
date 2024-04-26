#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

CcAffordancePlannerRos::CcAffordancePlannerRos(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options), // Use new class name
      node_logger_(this->get_logger()),
      plan_and_viz_ss_name_("/moveit_plan_and_viz_server")
{
    // Extract ros setup parameter from nodeoptions
    traj_execution_as_name_ = this->get_parameter("cca_robot_as").as_string();
    robot_description_parameter_ = this->get_parameter("cca_robot_description_parameter").as_string();
    planning_group_ = this->get_parameter("cca_planning_group").as_string();
    rviz_fixed_frame_ = this->get_parameter("rviz_fixed_frame").as_string();
    const std::string joint_states_topic = this->get_parameter("cca_joint_states_topic").as_string();
    const std::string robot_name = this->get_parameter("cca_robot").as_string();

    // Extract robot config info
    const std::string robot_config_file_path = CcAffordancePlannerRos::get_cc_affordance_robot_description_(robot_name);
    const AffordanceUtil::RobotConfig &robotConfig = AffordanceUtil::robot_builder(robot_config_file_path);
    robot_slist_ = robotConfig.Slist;
    M_ = robotConfig.M;
    ref_frame_ = robotConfig.ref_frame_name;
    tool_frame_ = robotConfig.tool_name;
    joint_names_ = robotConfig.joint_names;

    // Initialize clients and subscribers
    traj_execution_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, traj_execution_as_name_);
    joint_states_sub_ = this->create_subscription<JointState>(
        joint_states_topic, 1000, std::bind(&CcAffordancePlannerRos::joint_states_cb_, this, std::placeholders::_1));

    // Initialize clients and subscribers
    plan_and_viz_client_ = this->create_client<MoveItPlanAndViz>(plan_and_viz_ss_name_);

    // Construct buffer to lookup affordance location from apriltag using tf data
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool CcAffordancePlannerRos::run_cc_affordance_planner(const Eigen::Vector3d &w_aff, const Eigen::Vector3d &q_aff,
                                                       const Eigen::VectorXd &sec_goal, const double &aff_step,
                                                       const int &gripper_control_par_tau, const double &accuracy)
{
    // Compute affordance screw
    const Eigen::Matrix<double, 6, 1> aff_screw = AffordanceUtil::get_screw(w_aff, q_aff); // compute affordance screw

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

    const double aff_goal = sec_goal.tail(1)(0);
    ccAffordancePlanner.p_aff_step_deltatheta_a = sign_of(aff_goal) * abs(aff_step);
    ccAffordancePlanner.p_accuracy = accuracy;

    PlannerResult plannerResult = ccAffordancePlanner.affordance_stepper(cc_slist, sec_goal, gripper_control_par_tau);

    // Print planner result
    std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
    if (plannerResult.success)
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with "
                                             << plannerResult.traj_full_or_partial << " solution, and planning took "
                                             << plannerResult.planning_time.count() << " microseconds");
    }
    else
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
    }

    // Visualize and execute trajectory
    return visualize_and_execute_trajectory_(solution);
}

bool CcAffordancePlannerRos::run_cc_affordance_planner(const Eigen::Vector3d &w_aff,
                                                       const std::string &apriltag_frame_name,
                                                       const Eigen::VectorXd &sec_goal, const double &aff_step,
                                                       const int &gripper_control_par_tau, const double &accuracy)
{

    // Get affordance screw from april tag
    const Eigen::Isometry3d tag_htm = AffordanceUtilROS::get_htm(ref_frame_, apriltag_frame_name, *tf_buffer_);
    const Eigen::Vector3d q_aff = tag_htm.translation(); // location of the tag
    RCLCPP_INFO_STREAM(node_logger_, "Here is the affordance frame location. Ensure it's not empty and makes sense:\n"
                                         << q_aff);
    const Eigen::Matrix<double, 6, 1> aff_screw = AffordanceUtil::get_screw(w_aff, q_aff); // compute affordance screw

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

    const double aff_goal = sec_goal.tail(1)(0);
    ccAffordancePlanner.p_aff_step_deltatheta_a = sign_of(aff_goal) * abs(aff_step);
    ccAffordancePlanner.p_accuracy = accuracy;

    PlannerResult plannerResult = ccAffordancePlanner.affordance_stepper(cc_slist, sec_goal, gripper_control_par_tau);

    // Print planner result
    std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
    if (plannerResult.success)
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with "
                                             << plannerResult.traj_full_or_partial << " solution, and planning took "
                                             << plannerResult.planning_time.count() << " microseconds");
    }
    else
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
    }

    // Visualize and execute trajectory
    return visualize_and_execute_trajectory_(solution);
}

// Returns full path to the yaml file containing cc affordance robot description
std::string CcAffordancePlannerRos::get_cc_affordance_robot_description_(const std::string &robot_name)
{
    const std::string package_name = "cca_" + robot_name;            // get package name from the parameter server
    const std::string rel_dir = "/config/";                          // relative directory where yaml file is located
    const std::string filename = package_name + "_description.yaml"; // yaml file name
    return AffordanceUtilROS::get_filepath_inside_pkg(package_name, rel_dir, filename);
}

// Callback function for the joint_states subscriber
void CcAffordancePlannerRos::joint_states_cb_(const JointState::SharedPtr msg)
{
    joint_states_ = AffordanceUtilROS::get_ordered_joint_states(
        msg,
        joint_names_); // Takes care of filtering and ordering the joint_states
}

// Function to read robot joint states at the start of the affordance
Eigen::VectorXd CcAffordancePlannerRos::get_aff_start_joint_states_()
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
bool CcAffordancePlannerRos::visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory)
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
            RCLCPP_INFO_STREAM(node_logger_, "You canceled trajectory execution");
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
    RCLCPP_INFO(node_logger_, "%s action server call concluded", traj_execution_as_name_.c_str());
    rclcpp::shutdown();
}

// Callback to process traj_exection_as goal response
void CcAffordancePlannerRos::traj_execution_goal_response_callback_(
    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> future)
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
