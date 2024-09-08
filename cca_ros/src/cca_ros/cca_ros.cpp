#include <cca_ros/cca_ros.hpp>

namespace cca_ros
{
// Utility function to convert Eigen::VectorXd to a string
std::string vectorToString(const Eigen::VectorXd &vec)
{
    std::stringstream ss;
    ss << "[";
    for (int i = 0; i < vec.size(); ++i)
    {
        if (i > 0)
        {
            ss << ", ";
        }
        ss << vec[i];
    }
    ss << "]";
    return ss.str();
}

// Constructor for CcaRos, initializes the node and sets up required parameters and clients.
CcaRos::CcaRos(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options),
      node_logger_(this->get_logger()),   // Logger for the node
      viz_ss_name_("/cca_ros_viz_server") // Name of the MoveIt Plan and Visualization server
{
    // Extract necessary parameters for ROS setup and robot configuration
    robot_traj_execution_as_name_ = this->get_parameter("cca_robot_as").as_string();
    gripper_traj_execution_as_name_ = this->get_parameter_or<std::string>("cca_gripper_as", ""); // optional

    robot_description_parameter_ = this->get_parameter("cca_robot_description_parameter").as_string();
    planning_group_ = this->get_parameter("cca_planning_group").as_string();
    rviz_fixed_frame_ = this->get_parameter("rviz_fixed_frame").as_string();
    const std::string joint_states_topic = this->get_parameter("cca_joint_states_topic").as_string();
    const std::string robot_name = this->get_parameter("cca_robot").as_string();

    // Get the path for robot configuration file
    const std::string robot_config_file_path = CcaRos::get_cc_affordance_robot_description_(robot_name);

    // Load robot configuration
    try
    {
        const affordance_util::RobotConfig &robotConfig = affordance_util::robot_builder(robot_config_file_path);
        robot_slist_ = robotConfig.Slist;             // Robot screw axes
        M_ = robotConfig.M;                           // Home configuration matrix
        ref_frame_ = robotConfig.ref_frame_name;      // Reference frame
        tool_frame_ = robotConfig.tool_name;          // Tool frame
        robot_joint_names_ = robotConfig.joint_names; // Robot joint names
        gripper_joint_names_ = {"arm0_fingers"};      // Gripper joint names
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_logger_, "Exception while building robot configuration: %s", e.what());
    }

    // Initialize action clients and subscribers
    robot_traj_execution_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(this, robot_traj_execution_as_name_);
    if (!gripper_traj_execution_as_name_.empty())
    {
        // Only initialize if the gripper as name is provided
        gripper_traj_execution_client_ =
            rclcpp_action::create_client<FollowJointTrajectory>(this, gripper_traj_execution_as_name_);
    }
    viz_client_ = this->create_client<CcaRosViz>(viz_ss_name_);
    joint_states_sub_ = this->create_subscription<JointState>(
        joint_states_topic, 1000, std::bind(&CcaRos::joint_states_cb_, this, std::placeholders::_1));

    // Setup TF buffer and listener for affordance location from apriltag
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// Destructor for CcaRos, cleans up.
CcaRos::~CcaRos()
{
    rclcpp::shutdown();

    if (result_status_thread_.joinable())
    {
        result_status_thread_.join();
    }
}

// Runs the affordance planner for a single task and config.
bool CcaRos::run_cc_affordance_planner(const cc_affordance_planner::PlannerConfig &planner_config,
                                       const cc_affordance_planner::TaskDescription &taskDescription,
                                       const std::shared_ptr<Status> status, const KinematicState &startConfig)
{
    status_ = status;
    *status_ = Status::PROCESSING;

    // Validate input
    try
    {
        this->validate_input_(taskDescription);
    }
    catch (const std::invalid_argument &e)
    {
        RCLCPP_ERROR(node_logger_, "Error in input validation: %s", e.what());
        *status_ = Status::FAILED;
        return false;
    }

    // Copy task description and start config for potential modifications
    cc_affordance_planner::TaskDescription task_description = taskDescription;
    Eigen::VectorXd robot_start_config = startConfig.robot;
    double gripper_start_config = startConfig.gripper;
    const bool includes_gripper_trajectory = !std::isnan(task_description.goal.gripper);

    // Lookup affordance location if the tag frame is specified
    if (!task_description.affordance_info.location_frame.empty())
    {
        const Eigen::Isometry3d aff_htm =
            affordance_util_ros::get_htm(ref_frame_, task_description.affordance_info.location_frame, *tf_buffer_);
        if (aff_htm.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(node_logger_, "Could not lookup %s frame. Shutting down.",
                         task_description.affordance_info.location_frame.c_str());
            *status_ = Status::FAILED;
            return false;
        }
        task_description.affordance_info.location = aff_htm.translation();
    }

    // Get joint states if start configuration is empty
    if (robot_start_config.size() == 0)
    {
        auto state = read_joint_states_();
        robot_start_config = state.robot;
    }

    if (includes_gripper_trajectory && std::isnan(gripper_start_config))
    {
        auto state = read_joint_states_();
        gripper_start_config = state.gripper;
    }

    // Prepare robot description for planning
    affordance_util::RobotDescription robot_description;
    robot_description.slist = robot_slist_;
    robot_description.M = M_;
    robot_description.joint_states = robot_start_config;
    robot_description.gripper_state = gripper_start_config;

    // Create and run the planner interface
    cc_affordance_planner::CcAffordancePlannerInterface ccAffordancePlannerInterface(planner_config);
    cc_affordance_planner::PlannerResult plannerResult;
    try
    {
        plannerResult = ccAffordancePlannerInterface.generate_joint_trajectory(robot_description, task_description);
    }
    catch (const std::invalid_argument &e)
    {
        RCLCPP_ERROR(node_logger_, "Planner returned exception: %s", e.what());
    }

    // Handle planner result
    if (plannerResult.success)
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with update trail '"
                                             << plannerResult.update_trail << "' and took "
                                             << plannerResult.planning_time.count() << " microseconds.");
        RCLCPP_WARN(node_logger_, "Joint trajectory size: %zu", plannerResult.joint_trajectory.size());
        if ((plannerResult.trajectory_description == cc_affordance_planner::TrajectoryDescription::PARTIAL) &&
            ((task_description.trajectory_density - plannerResult.joint_trajectory.size()) > 2))
        {
            RCLCPP_ERROR(node_logger_, "Trajectory description: PARTIAL.");
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

    // Visualize and execute the planned trajectory
    return visualize_and_execute_trajectory_(plannerResult.joint_trajectory, task_description.affordance_info.axis,
                                             task_description.affordance_info.location,
                                             plannerResult.includes_gripper_trajectory);
}

// Runs the affordance planner for multiple tasks and configurations.
bool CcaRos::run_cc_affordance_planner(const std::vector<cc_affordance_planner::PlannerConfig> &planner_configs,
                                       const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions,
                                       const std::shared_ptr<Status> status, const KinematicState &startConfig)
{
    status_ = status;
    *status_ = Status::PROCESSING;

    // Validate input
    try
    {
        this->validate_input_(planner_configs, task_descriptions);
    }
    catch (const std::invalid_argument &e)
    {
        RCLCPP_ERROR(node_logger_, "Error in input validation: %s", e.what());
        *status_ = Status::FAILED;
        return false;
    }

    const bool includes_gripper_trajectory = !std::isnan(task_descriptions.front().goal.gripper);
    Eigen::VectorXd robot_start_config = startConfig.robot;
    double gripper_start_config = startConfig.gripper;

    // Get joint states if start configuration is empty
    if (robot_start_config.size() == 0)
    {
        auto state = read_joint_states_();
        robot_start_config = state.robot;
    }

    if (includes_gripper_trajectory && std::isnan(gripper_start_config))
    {
        auto state = read_joint_states_();
        gripper_start_config = state.gripper;
    }
    // Prepare robot description
    affordance_util::RobotDescription robot_description;
    robot_description.slist = robot_slist_;
    robot_description.M = M_;
    robot_description.joint_states = robot_start_config;
    robot_description.gripper_state = gripper_start_config;

    std::vector<Eigen::VectorXd> solution;

    // Iterate through all planner configurations
    for (size_t i = 0; i < planner_configs.size(); ++i)
    {
        cc_affordance_planner::TaskDescription task_description = task_descriptions[i];

        // Lookup affordance location if specified
        if (!task_description.affordance_info.location_frame.empty())
        {
            const Eigen::Isometry3d aff_htm =
                affordance_util_ros::get_htm(ref_frame_, task_description.affordance_info.location_frame, *tf_buffer_);
            if (aff_htm.matrix().isApprox(Eigen::Matrix4d::Identity()))
            {
                RCLCPP_ERROR(node_logger_, "Could not lookup %s frame. Shutting down.",
                             task_description.affordance_info.location_frame.c_str());
                *status_ = Status::FAILED;
                return false;
            }
            task_description.affordance_info.location = aff_htm.translation();
        }

        // Create planner and run
        cc_affordance_planner::CcAffordancePlannerInterface ccAffordancePlannerInterface(planner_configs[i]);
        cc_affordance_planner::PlannerResult plannerResult;
        try
        {
            plannerResult = ccAffordancePlannerInterface.generate_joint_trajectory(robot_description, task_description);
        }
        catch (const std::invalid_argument &e)
        {
            RCLCPP_ERROR(node_logger_, "Planner returned exception: %s", e.what());
        }

        // Handle planner result
        if (plannerResult.success)
        {
            RCLCPP_WARN(node_logger_, "Joint trajectory size: %zu", plannerResult.joint_trajectory.size());
            if ((plannerResult.trajectory_description == cc_affordance_planner::TrajectoryDescription::PARTIAL) &&
                ((task_description.trajectory_density - plannerResult.joint_trajectory.size()) > 2))

            {
                RCLCPP_ERROR(node_logger_, "Partial solution at task %zu.", i);
                *status_ = Status::FAILED;
                return false;
            }
        }
        else
        {
            RCLCPP_WARN(node_logger_, "Planner did not find a solution for task %zu", i);
            *status_ = Status::FAILED;
            return false;
        }

        // Append joint trajectory to solution
        solution.insert(solution.end(), plannerResult.joint_trajectory.begin(), plannerResult.joint_trajectory.end());

        // Update robot description with the end point of the current trajectory
        robot_description.joint_states = plannerResult.joint_trajectory.back().head(robot_start_config.size());
        robot_description.gripper_state = plannerResult.joint_trajectory.back()[robot_joint_names_.size()];
    }

    // Visualize and execute the complete trajectory
    return visualize_and_execute_trajectory_(solution, task_descriptions.back().affordance_info.axis,
                                             task_descriptions.back().affordance_info.location,
                                             includes_gripper_trajectory);
}

// Helper function to validate input
void CcaRos::validate_input_(const cc_affordance_planner::TaskDescription &task_description)
{
    RCLCPP_INFO(node_logger_, "Gripper goal, %f", task_description.goal.gripper);
    RCLCPP_INFO(node_logger_, "AS name, %s", gripper_traj_execution_as_name_.c_str());
    RCLCPP_INFO(node_logger_, "Gripper goal bool: %s", !std::isnan(task_description.goal.gripper) ? "true" : "false");
    RCLCPP_INFO(node_logger_, "AS bool: %s", gripper_traj_execution_as_name_.empty() ? "true" : "false");

    if (!std::isnan(task_description.goal.gripper) && gripper_traj_execution_as_name_.empty())
    {
        throw std::invalid_argument("Task description: `goal.gripper` is specified, but `cca_gripper_as` parameter is "
                                    "not set up in the `cca_<robot>_ros_setup.yaml` file.");
    }
}

// Helper function to validate input
void CcaRos::validate_input_(const std::vector<cc_affordance_planner::PlannerConfig> &planner_configs,
                             const std::vector<cc_affordance_planner::TaskDescription> &task_descriptions)
{
    // Ensure planner configs and task descriptions are of the same size
    if (planner_configs.size() != task_descriptions.size() || planner_configs.empty())
    {
        throw std::invalid_argument("Planner configs and task descriptions: Must be of the same size and non-empty.");
    }

    // Ensure gripper goals are consistent across all tasks
    bool first_gripper_goal_status = !std::isnan(task_descriptions.front().goal.gripper);
    this->validate_input_(
        task_descriptions.front()); // At the moment, we're just validating gripper info so, just check the first one.

    for (const auto &task_description : task_descriptions)
    {
        bool gripper_goal_status = !std::isnan(task_description.goal.gripper);

        if (gripper_goal_status != first_gripper_goal_status)
        { // Check for logical inequivalence
            throw std::invalid_argument(
                "Task description: Inconsistent gripper goal across tasks. If one task considers the gripper goal, "
                "then all tasks must have the gripper goal set.");
        }
    }
}

// Helper function to get the full path to the robot description file.
std::string CcaRos::get_cc_affordance_robot_description_(const std::string &robot_name)
{
    const std::string package_name = "cca_" + robot_name;
    const std::string rel_dir = "/config/";
    const std::string filename = package_name + "_description.yaml";
    return affordance_util_ros::get_filepath_inside_pkg(package_name, rel_dir, filename);
}

// Callback for joint_states topic.
void CcaRos::joint_states_cb_(const JointState::SharedPtr msg)
{

    robot_joint_states_ = affordance_util_ros::get_ordered_joint_states(msg, robot_joint_names_);
    gripper_joint_states_ = affordance_util_ros::get_ordered_joint_states(msg, gripper_joint_names_);
}

// Retrieve robot joint states at the start of the affordance.
KinematicState CcaRos::read_joint_states_()
{
    robot_joint_states_.positions.conservativeResize(robot_joint_names_.size());
    gripper_joint_states_.positions.conservativeResize(gripper_joint_names_.size());
    robot_joint_states_.positions.setConstant(std::numeric_limits<double>::quiet_NaN());
    gripper_joint_states_.positions.setConstant(std::numeric_limits<double>::quiet_NaN());

    rclcpp::sleep_for(std::chrono::milliseconds(500)); // Give time to read from callback
    if ((robot_joint_states_.positions.hasNaN()) || (gripper_joint_states_.positions.hasNaN()))
    {
        RCLCPP_ERROR(node_logger_, "Could not read joint states");
    }

    return KinematicState{robot_joint_states_.positions, gripper_joint_states_.positions[0]};
}

// Visualizes and executes the given trajectory.
bool CcaRos::visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory,
                                               const Eigen::VectorXd &w_aff, const Eigen::VectorXd &q_aff,
                                               const bool includes_gripper_trajectory)
{
    std::vector<Eigen::VectorXd> gripper_trajectory;
    std::cout << "Here is the gripper trajectory:" << std::endl;
    if (includes_gripper_trajectory)
    {
        for (const auto &point : trajectory)
        {
            Eigen::VectorXd gripper_point = (Eigen::VectorXd(1) << point[robot_joint_names_.size()]).finished();
            gripper_trajectory.push_back(gripper_point);
            std::cout << gripper_point[0] << ", ";
        }
        std::cout << std::endl;
    }

    // Create ROS message for trajectory and visualization
    const double traj_time_step = 0.3;
    const control_msgs::action::FollowJointTrajectory_Goal goal =
        affordance_util_ros::follow_joint_trajectory_msg_builder(trajectory, Eigen::VectorXd::Zero(6),
                                                                 robot_joint_names_, traj_time_step);

    auto viz_serv_req = std::make_shared<CcaRosViz::Request>();
    viz_serv_req->joint_traj = goal.trajectory;
    viz_serv_req->aff_screw_axis = {w_aff[0], w_aff[1], w_aff[2]};
    viz_serv_req->aff_location = {q_aff[0], q_aff[1], q_aff[2]};
    viz_serv_req->ref_frame = ref_frame_;
    viz_serv_req->tool_frame = tool_frame_;
    viz_serv_req->planning_group = planning_group_;
    viz_serv_req->robot_description = robot_description_parameter_;
    viz_serv_req->rviz_fixed_frame = rviz_fixed_frame_;

    // Wait for visualization service
    while (!viz_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node_logger_, "Interrupted while waiting for %s service. Exiting.", viz_ss_name_.c_str());
            return false;
        }
        RCLCPP_INFO(node_logger_, " %s service not available, waiting again...", viz_ss_name_.c_str());
    }

    // Send the request to visualize
    auto result_future = viz_client_->async_send_request(viz_serv_req);
    RCLCPP_INFO(node_logger_, "Waiting on %s service to complete", viz_ss_name_.c_str());
    auto response = result_future.get();

    // Check if the visualization service succeeded
    if (response->success)
    {
        RCLCPP_INFO(node_logger_, " %s service succeeded", viz_ss_name_.c_str());

        auto robot_send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        robot_send_goal_options.goal_response_callback =
            std::bind(&CcaRos::robot_traj_execution_goal_response_callback_, this, std::placeholders::_1);
        robot_send_goal_options.result_callback =
            std::bind(&CcaRos::robot_traj_execution_result_callback_, this, std::placeholders::_1);

        RCLCPP_INFO(node_logger_, "DEBUG BEFORE FLAGGGGGGGGGG");
        if (includes_gripper_trajectory)
        {
            auto gripper_send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            gripper_send_goal_options.goal_response_callback =
                std::bind(&CcaRos::gripper_traj_execution_goal_response_callback_, this, std::placeholders::_1);
            gripper_send_goal_options.result_callback =
                std::bind(&CcaRos::gripper_traj_execution_result_callback_, this, std::placeholders::_1);

            RCLCPP_INFO(node_logger_, "DEBUG FLAGGGGGGGGGG");
            // Start checking the result status in a separate thread
            result_status_thread_ = std::thread(&CcaRos::check_robot_and_gripper_result_status_, this);
            RCLCPP_INFO(node_logger_, "DEBUG THREAD AFTER FLAGGGGGGGGGG");

            return (execute_trajectory_(robot_traj_execution_client_, robot_send_goal_options,
                                        robot_traj_execution_as_name_, robot_joint_names_, trajectory, 0.3)) &&
                   (execute_trajectory_(gripper_traj_execution_client_, gripper_send_goal_options,
                                        gripper_traj_execution_as_name_, gripper_joint_names_, gripper_trajectory,
                                        0.1));
        }
        else
        {
            robot_result_status_ = status_;
            return execute_trajectory_(robot_traj_execution_client_, robot_send_goal_options,
                                       robot_traj_execution_as_name_, robot_joint_names_, trajectory, 0.3);
        }
    }
    else
    {
        RCLCPP_ERROR(node_logger_, "%s service call failed", viz_ss_name_.c_str());
        return false;
    }
}

// Executes the planned trajectory.
bool CcaRos::execute_trajectory_(rclcpp_action::Client<FollowJointTrajectory>::SharedPtr &traj_execution_client,
                                 rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options,
                                 const std::string &traj_execution_as_name, const std::vector<std::string> &joint_names,
                                 const std::vector<Eigen::VectorXd> &trajectory, const double &traj_time_step)
{
    /* if (traj_execution_as_name == robot_traj_execution_as_name_) */
    /* { */
    /*     // Ask for confirmation before executing the trajectory */
    /*     RCLCPP_INFO_STREAM(node_logger_, "Ready to execute the trajectory? Press 'y' to confirm"); */
    /*     std::string execution_conf; */
    /*     std::cin >> execution_conf; */

    /*     if (execution_conf != "y" && execution_conf != "Y") */
    /*     { */
    /*         RCLCPP_INFO_STREAM(node_logger_, "Trajectory execution was canceled"); */
    /*         return false; */
    /*     } */
    /* } */

    // Convert the solution trajectory to ROS message
    /* const double traj_time_step = 0.3; */
    const control_msgs::action::FollowJointTrajectory_Goal goal =
        affordance_util_ros::follow_joint_trajectory_msg_builder(trajectory, Eigen::VectorXd::Zero(6), joint_names,
                                                                 traj_time_step);

    // Wait for the action server to be ready
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

// Callback to handle the result of robot trajectory execution
void CcaRos::robot_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result)
{
    // Analyze result
    *robot_result_status_ = this->analyze_as_result_(result.code, robot_traj_execution_as_name_);
}

// Callback to handle the goal response for robot trajectory execution
void CcaRos::robot_traj_execution_goal_response_callback_(const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
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

// Callback to handle the result of gripper trajectory execution
void CcaRos::gripper_traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result)
{
    // Analyze result
    *gripper_result_status_ = this->analyze_as_result_(result.code, gripper_traj_execution_as_name_);
}

// Callback to handle the goal response for gripper trajectory execution
void CcaRos::gripper_traj_execution_goal_response_callback_(
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

Status CcaRos::analyze_as_result_(const rclcpp_action::ResultCode &result_code, const std::string &as_name)
{

    Status result_status = cca_ros::Status::UNKNOWN;

    switch (result_code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        result_status = Status::SUCCEEDED;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_logger_, "%s action server goal was aborted", as_name.c_str());
        result_status = Status::FAILED;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_logger_, "%s action server goal was canceled", as_name.c_str());
        result_status = Status::FAILED;
        break;
    default:
        RCLCPP_ERROR(node_logger_, "%s action server returned unknown result code", as_name.c_str());
        result_status = Status::FAILED;
        break;
    }

    RCLCPP_INFO(node_logger_, "%s action server call concluded", as_name.c_str());

    return result_status;
}

void CcaRos::check_robot_and_gripper_result_status_()
{
    // Start statuses as unknown
    robot_result_status_ = std::make_shared<cca_ros::Status>(cca_ros::Status::PROCESSING);
    gripper_result_status_ = std::make_shared<cca_ros::Status>(cca_ros::Status::PROCESSING);
    while (rclcpp::ok())
    {
        if (*robot_result_status_ != cca_ros::Status::PROCESSING &&
            *gripper_result_status_ != cca_ros::Status::PROCESSING)
        {

            // Both pointers are not in PROCESSING status, check their values
            std::lock_guard<std::mutex> lock(status_mutex_); // Lock the mutex before modifying status_
            if (*robot_result_status_ == cca_ros::Status::SUCCEEDED &&
                *gripper_result_status_ == cca_ros::Status::SUCCEEDED)
            {
                *status_ = cca_ros::Status::SUCCEEDED;
            }
            else
            {
                *status_ = cca_ros::Status::FAILED;
            }
            return; // Exit
        }

        // Sleep for a short duration to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

void CcaRos::cleanup_between_calls()
{
    if (result_status_thread_.joinable())
    {
        result_status_thread_.join();
    }
}
} // namespace cca_ros
