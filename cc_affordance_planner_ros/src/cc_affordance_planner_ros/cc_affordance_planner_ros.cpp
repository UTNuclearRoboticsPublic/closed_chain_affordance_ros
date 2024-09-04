#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

namespace cc_affordance_planner_ros
{
CcAffordancePlannerRos::CcAffordancePlannerRos(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options), // Use new class name
      node_logger_(this->get_logger()),
      plan_and_viz_ss_name_("/moveit_plan_and_viz_server")
{
    // Extract CC Affordance parameters regarding ROS-related setup
    robot_traj_execution_as_name_ = this->get_parameter("cca_robot_as").as_string();
    gripper_traj_execution_as_name_ = this->get_parameter("cca_robot_gripper_as").as_string();
    robot_description_parameter_ = this->get_parameter("cca_robot_description_parameter").as_string();
    planning_group_ = this->get_parameter("cca_planning_group").as_string();
    rviz_fixed_frame_ = this->get_parameter("rviz_fixed_frame").as_string();
    const std::string joint_states_topic = this->get_parameter("cca_joint_states_topic").as_string();
    const std::string robot_name = this->get_parameter("cca_robot").as_string();

    // Extract robot config info
    const std::string robot_config_file_path = CcAffordancePlannerRos::get_cc_affordance_robot_description_(robot_name);
    try
    {
        const affordance_util::RobotConfig &robotConfig = affordance_util::robot_builder(robot_config_file_path);
        robot_slist_ = robotConfig.Slist;
        M_ = robotConfig.M;
        ref_frame_ = robotConfig.ref_frame_name;
        tool_frame_ = robotConfig.tool_name;
        robot_joint_names_ = robotConfig.joint_names;
        gripper_joint_names_ = {"arm0_fingers"};
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Exception while building robot configuration: %s", e.what());
    }

    // Initialize clients and subscribers
    robot_traj_execution_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(this, robot_traj_execution_as_name_);
    gripper_traj_execution_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(this, gripper_traj_execution_as_name_);
    plan_and_viz_client_ = this->create_client<MoveItPlanAndViz>(plan_and_viz_ss_name_);
    joint_states_sub_ = this->create_subscription<JointState>(
        joint_states_topic, 1000, std::bind(&CcAffordancePlannerRos::joint_states_cb_, this, std::placeholders::_1));

    // Construct buffer to lookup affordance location from apriltag using tf data
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool CcAffordancePlannerRos::run_cc_affordance_planner(const cc_affordance_planner::PlannerConfig &planner_config,
                                                       const cc_affordance_planner::TaskDescription &taskDescription,
                                                       const std::shared_ptr<Status> status,
                                                       const Eigen::VectorXd &robotStartConfig)
{
    status_ = status;
    *status_ = Status::PROCESSING;

    // Copy task description and robot start config in case we need to make modifications before calling the planner
    cc_affordance_planner::TaskDescription task_description = taskDescription;
    Eigen::VectorXd robot_start_config = robotStartConfig;

    // If tag frame is specified then, we lookup affordance location from tag
    if (!task_description.affordance_info.location_frame.empty())
    {
        const Eigen::Isometry3d aff_htm =
            affordance_util_ros::get_htm(ref_frame_, task_description.affordance_info.location_frame, *tf_buffer_);
        if (aff_htm.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.",
                         task_description.affordance_info.location_frame.c_str());
            *status_ = Status::FAILED;
            return false;
        }
        task_description.affordance_info.location = aff_htm.translation();
    }

    Eigen::VectorXd gripper_start_config;
    // Get joint states at the start configuration of the affordance
    if (robot_start_config.size() == 0) // Non-zero if testing or planning without the joint_states topic
    {
        auto start_config = get_aff_start_joint_states_();
        robot_start_config = start_config.first;
        gripper_start_config = start_config.second;
    }

    // Fill out robot description
    affordance_util::RobotDescription robot_description;
    robot_description.slist = robot_slist_;
    robot_description.M = M_;
    robot_description.joint_states = robot_start_config;
    robot_description.gripper_state = gripper_start_config[0];

    // Construct the planner interface object with given configuration
    cc_affordance_planner::CcAffordancePlannerInterface ccAffordancePlannerInterface(planner_config);

    // Run the planner
    cc_affordance_planner::PlannerResult plannerResult;
    try
    {
        plannerResult = ccAffordancePlannerInterface.generate_joint_trajectory(robot_description, task_description);
    }
    catch (const std::invalid_argument &e)
    {
        RCLCPP_ERROR(node_logger_, "Planner returned exception: %s", e.what());
    }

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
            RCLCPP_ERROR(node_logger_, "Trajectory description: PARTIAL.");
            *status_ = Status::FAILED;
            return false;
        default:
            RCLCPP_ERROR(node_logger_, "Trajectory description: UNSET.");
            *status_ = Status::FAILED;
            return false;
        }
    }
    else
    {
        RCLCPP_WARN(node_logger_, "Planner did not find a solution");
        *status_ = Status::FAILED;
        return false;
    }

    // Visualize and execute trajectory
    return visualize_and_execute_trajectory_(solution, task_description.affordance_info.axis,
                                             task_description.affordance_info.location,
                                             plannerResult.includes_gripper_trajectory);
    // Execute trajectory
    /* return execute_trajectory_(solution); */
}

bool CcAffordancePlannerRos::run_cc_affordance_planner(
    const std::vector<cc_affordance_planner::PlannerConfig> &planner_configs,
    const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions, const std::shared_ptr<Status> status,
    const Eigen::VectorXd &robotStartConfig)
{
    status_ = status;
    *status_ = Status::PROCESSING;

    if ((planner_configs.size() != task_descriptions.size()) || planner_configs.empty())
    {
        RCLCPP_ERROR(node_logger_, "Planner config and task descriptions must be of the same size and non-empty.");
        return false;
    }

    // Copy robot start config in case we need to make modifications before calling the planner
    Eigen::VectorXd robot_start_config = robotStartConfig;

    Eigen::VectorXd gripper_start_config;
    // Get joint states at the start configuration of the affordance
    if (robot_start_config.size() == 0) // Non-zero if testing or planning without the joint_states topic
    {
        auto start_config = get_aff_start_joint_states_();
        robot_start_config = start_config.first;
        gripper_start_config = start_config.second;
    }

    // Fill out robot description
    affordance_util::RobotDescription robot_description;
    robot_description.slist = robot_slist_;
    robot_description.M = M_;
    robot_description.joint_states = robot_start_config;
    robot_description.gripper_state = gripper_start_config[0];

    std::vector<Eigen::VectorXd> solution;
    cc_affordance_planner::TaskDescription task_description;

    bool includes_gripper_trajectory = false;
    size_t i = 0;
    for (const auto &planner_config : planner_configs)
    {
        // Copy task description in case we need to make modifications before calling the planner
        task_description = task_descriptions[i];

        // If tag frame is specified then, we lookup affordance location from tag
        if (!task_description.affordance_info.location_frame.empty())
        {
            const Eigen::Isometry3d aff_htm =
                affordance_util_ros::get_htm(ref_frame_, task_description.affordance_info.location_frame, *tf_buffer_);
            if (aff_htm.matrix().isApprox(Eigen::Matrix4d::Identity()))
            {
                RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Shutting down.",
                             task_description.affordance_info.location_frame.c_str());
                *status_ = Status::FAILED;
                return false;
            }
            task_description.affordance_info.location = aff_htm.translation();
        }

        // Construct the planner interface object with given configuration
        cc_affordance_planner::CcAffordancePlannerInterface ccAffordancePlannerInterface(planner_config);

        // Run the planner
        cc_affordance_planner::PlannerResult plannerResult;
        try
        {
            plannerResult = ccAffordancePlannerInterface.generate_joint_trajectory(robot_description, task_description);
            includes_gripper_trajectory = plannerResult.includes_gripper_trajectory;
        }
        catch (const std::invalid_argument &e)
        {
            RCLCPP_ERROR(node_logger_, "Planner returned exception: %s", e.what());
        }

        // Print planner result
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
                RCLCPP_ERROR(node_logger_, "Terminating planning due to the solution for %zuth task being partial.", i);
                *status_ = Status::FAILED;
                return false;
            default:
                RCLCPP_ERROR(node_logger_, "Trajectory description: UNSET.");
                *status_ = Status::FAILED;
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(node_logger_, "Planner did not find a solution for %zuth task", i);
            *status_ = Status::FAILED;
            return false;
        }

        // Append the solution
        solution.insert(solution.end(), plannerResult.joint_trajectory.begin(), plannerResult.joint_trajectory.end());

        // Next iteration updates

        // Update the robot description start state for the next motion with the end point from the previous
        // motion trajectory
        robot_description.joint_states = plannerResult.joint_trajectory.back().head(robot_start_config.size());
        robot_description.gripper_state = plannerResult.joint_trajectory.back()[robot_joint_names_.size()];

        i++;
    }

    // Visualize and execute trajectory
    return visualize_and_execute_trajectory_(solution, task_description.affordance_info.axis,
                                             task_description.affordance_info.location, includes_gripper_trajectory);
    // Execute trajectory
    /* return execute_trajectory_(solution); */
}

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
    robot_joint_states_ = affordance_util_ros::get_ordered_joint_states(
        msg,
        robot_joint_names_); // Takes care of filtering and ordering the joint_states
    gripper_joint_states_ = affordance_util_ros::get_ordered_joint_states(
        msg,
        gripper_joint_names_); // Takes care of filtering and ordering the joint_states
}

// Function to read robot joint states at the start of the affordance
std::pair<Eigen::VectorXd, Eigen::VectorXd> CcAffordancePlannerRos::get_aff_start_joint_states_()
{
    if ((robot_joint_states_.positions.size() == 0) || (gripper_joint_states_.positions.size() == 0))
    {
        RCLCPP_ERROR(node_logger_, "Could not read joint states");
    }

    // Set Eigen::VectorXd size
    robot_joint_states_.positions.conservativeResize(robot_joint_names_.size());
    gripper_joint_states_.positions.conservativeResize(gripper_joint_names_.size());

    return std::make_pair(robot_joint_states_.positions, gripper_joint_states_.positions);
}

// Function to visualize and execute planned trajectory
bool CcAffordancePlannerRos::visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory,
                                                               const Eigen::VectorXd &w_aff,
                                                               const Eigen::VectorXd &q_aff,
                                                               const bool includes_gripper_trajectory)
{

    std::vector<Eigen::VectorXd> gripper_trajectory;
    if (includes_gripper_trajectory)
    {
        // Extract gripper trajectory
        for (const auto &point : trajectory)
        {
            Eigen::VectorXd gripper_point = (Eigen::VectorXd(1) << point[robot_joint_names_.size()]).finished();
            gripper_trajectory.push_back(gripper_point);
        }
    }

    // Visualize trajectory in RVIZ
    // Convert the solution trajectory to ROS message type
    const double traj_time_step = 0.3;
    const control_msgs::action::FollowJointTrajectory_Goal goal =
        affordance_util_ros::follow_joint_trajectory_msg_builder(
            trajectory, Eigen::VectorXd::Zero(6), robot_joint_names_,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains the whole closed-chain trajectory

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

        if (includes_gripper_trajectory)
        {
            auto robot_send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            robot_send_goal_options.goal_response_callback = std::bind(
                &CcAffordancePlannerRos::robot_traj_execution_goal_response_callback_, this, std::placeholders::_1);
            robot_send_goal_options.result_callback =
                std::bind(&CcAffordancePlannerRos::robot_traj_execution_result_callback_, this, std::placeholders::_1);
            auto gripper_send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            gripper_send_goal_options.goal_response_callback = std::bind(
                &CcAffordancePlannerRos::gripper_traj_execution_goal_response_callback_, this, std::placeholders::_1);
            gripper_send_goal_options.result_callback = std::bind(
                &CcAffordancePlannerRos::gripper_traj_execution_result_callback_, this, std::placeholders::_1);
            return (this->execute_trajectory_(this->robot_traj_execution_client_, robot_send_goal_options,
                                              robot_traj_execution_as_name_, robot_joint_names_, trajectory)) &&
                   (this->execute_trajectory_(this->gripper_traj_execution_client_, gripper_send_goal_options,
                                              gripper_traj_execution_as_name_, gripper_joint_names_,
                                              gripper_trajectory));
        }
        else
        {

            auto robot_send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            robot_send_goal_options.goal_response_callback = std::bind(
                &CcAffordancePlannerRos::robot_traj_execution_goal_response_callback_, this, std::placeholders::_1);
            robot_send_goal_options.result_callback =
                std::bind(&CcAffordancePlannerRos::robot_traj_execution_result_callback_, this, std::placeholders::_1);
            return this->execute_trajectory_(robot_traj_execution_client_, robot_send_goal_options,
                                             robot_traj_execution_as_name_, robot_joint_names_, trajectory);
        }
    }
    else
    {
        RCLCPP_ERROR(node_logger_, "%s service call failed", plan_and_viz_ss_name_.c_str());
        return false;
    }
}

// Function to execute planned trajectory
bool CcAffordancePlannerRos::execute_trajectory_(
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr &traj_execution_client,
    rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options,
    const std::string &traj_execution_as_name, const std::vector<std::string> &joint_names,
    const std::vector<Eigen::VectorXd> &trajectory)
{
    // Execute trajectory on the real robot
    RCLCPP_INFO_STREAM(node_logger_, "Ready to execute the trajectory? y to confirm");
    std::string execution_conf;
    std::cin >> execution_conf;

    if (execution_conf != "y" && execution_conf != "Y")
    {
        RCLCPP_INFO_STREAM(node_logger_, "Trajectory execution was canceled");
        return false;
    }

    // Visualize trajectory in RVIZ
    // Convert the solution trajectory to ROS message type
    const double traj_time_step = 0.3;
    const control_msgs::action::FollowJointTrajectory_Goal goal =
        affordance_util_ros::follow_joint_trajectory_msg_builder(
            trajectory, Eigen::VectorXd::Zero(6), joint_names,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains qs data too

    // Send the goal to follow_joint_trajectory action server for execution
    if (!traj_execution_client->wait_for_action_server())
    {
        RCLCPP_ERROR(node_logger_, " %s action server not available after waiting", traj_execution_as_name.c_str());
        *status_ = Status::FAILED;
        return false;
    }

    RCLCPP_INFO(node_logger_, "Sending goal to %s action server", traj_execution_as_name.c_str());

    traj_execution_client->async_send_goal(goal, send_goal_options);
    return true;
}

// Callback to process traj_execution_as result
void CcAffordancePlannerRos::robot_traj_execution_result_callback_(
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
    RCLCPP_INFO(node_logger_, "%s action server call concluded", robot_traj_execution_as_name_.c_str());
}

// Callback to process traj_exection_as goal response
void CcAffordancePlannerRos::robot_traj_execution_goal_response_callback_(
    const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_logger_, "Goal was rejected by %s action server", robot_traj_execution_as_name_.c_str());
    }
    else
    {
        RCLCPP_INFO(node_logger_, "Goal accepted by %s action server, waiting for result",
                    robot_traj_execution_as_name_.c_str());
    }
}
// Callback to process traj_execution_as result
void CcAffordancePlannerRos::gripper_traj_execution_result_callback_(
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
    RCLCPP_INFO(node_logger_, "%s action server call concluded", gripper_traj_execution_as_name_.c_str());
}

// Callback to process traj_exection_as goal response
void CcAffordancePlannerRos::gripper_traj_execution_goal_response_callback_(
    const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_logger_, "Goal was rejected by %s action server", gripper_traj_execution_as_name_.c_str());
    }
    else
    {
        RCLCPP_INFO(node_logger_, "Goal accepted by %s action server, waiting for result",
                    gripper_traj_execution_as_name_.c_str());
    }
}
} // namespace cc_affordance_planner_ros
