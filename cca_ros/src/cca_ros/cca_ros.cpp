#include <algorithm>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <cca_ros_msgs/srv/detail/cca_ros_viz__struct.hpp>
#include <chrono>
#include <trajectory_msgs/msg/detail/joint_trajectory__struct.hpp>

namespace cca_ros
{

// Constructor for CcaRos, initializes the node and sets up required parameters and clients.
CcaRos::CcaRos(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options),
      node_logger_(this->get_logger()),   // Logger for the node
      viz_ss_name_("/cca_ros_viz_server") // Name of the MoveIt Plan and Visualization server
{
    // Extract necessary parameters for ROS setup and robot configuration
    robot_traj_execution_as_name_ = this->declare_parameter("cca_robot_as", rclcpp::ParameterType::PARAMETER_STRING).get<std::string>();
    gripper_traj_execution_as_name_ = this->declare_parameter("cca_gripper_as", ""); // optional
    robot_and_gripper_traj_execution_as_name_ =
        this->declare_parameter("cca_robot_and_gripper_as", ""); // optional

    const std::string joint_states_topic = this->declare_parameter("cca_joint_states_topic", rclcpp::ParameterType::PARAMETER_STRING).get<std::string>();
    const std::string robot_name = this->declare_parameter("cca_robot", rclcpp::ParameterType::PARAMETER_STRING).get<std::string>();
    const std::string build_robot_from = this->declare_parameter("cca_build_robot_from", rclcpp::ParameterType::PARAMETER_STRING).get<std::string>();

    if (build_robot_from != "yaml" && build_robot_from != "urdf") {
    	RCLCPP_ERROR(node_logger_, "Invalid value for the [cca_build_robot_from] parameter: %s. Possible options are yaml or urdf", build_robot_from.c_str());
    }

    // Get the path for robot configuration file
    const std::string robot_config_file_path = CcaRos::get_cc_affordance_robot_description_(robot_name, build_robot_from);

    affordance_util::RobotConfig robotConfig;


    // Load robot configuration
    try{

	if (build_robot_from=="yaml"){
	    robotConfig = affordance_util::robot_builder(robot_config_file_path);
	}
	else { // "urdf"
	    const affordance_util::RobotConfig &urdfConfig = affordance_util::extract_info_for_urdf_robot_builder(robot_config_file_path);

	    std::string robot_description;
	    try{
	    	robot_description = this->declare_parameter("robot_description", rclcpp::ParameterType::PARAMETER_STRING).get<std::string>();
	       }
            catch (const std::exception &e){
	        RCLCPP_ERROR(node_logger_, "Exception while loading robot_description param: %s", e.what());
	       }

	    robotConfig = affordance_util::robot_builder(robot_description, urdfConfig);
	}

    }
    catch (const std::exception &e){
	RCLCPP_ERROR(node_logger_, "Exception while building robot configuration: %s", e.what());
    }

    // Extract necessary info from robot config
    robot_slist_ = robotConfig.Slist;                         // Robot screw axes
    M_ = robotConfig.M;                                       // Home configuration matrix
    ref_frame_ = robotConfig.frame_names.ref;                 // Reference frame
    tool_frame_ = robotConfig.frame_names.tool;               // Tool frame
    robot_joint_names_ = robotConfig.joint_names.robot;       // Robot joint names
    gripper_joint_names_ = {robotConfig.joint_names.gripper}; // Gripper joint names

    // Initialize service/action clients and subscribers
    viz_client_ = this->create_client<CcaRosViz>(viz_ss_name_);
    this->initialize_action_clients_();
    joint_states_sub_ = this->create_subscription<JointState>(
        joint_states_topic, 1000, std::bind(&CcaRos::joint_states_cb_, this, std::placeholders::_1));

    // Setup TF buffer and listener to lookup affordance location from apriltag
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// Destructor for CcaRos, cleans up.
CcaRos::~CcaRos()
{
    rclcpp::shutdown();
}


#include "cca_ros/cca_ros.hpp"

cca_ros::PlanningResponse CcaRos::plan(const cca_ros::PlanningRequest &planning_request)
{
    // Run the vector-based version for a single element
    return this->plan(std::vector<cca_ros::PlanningRequest>{planning_request});
}

// Runs the affordance planner for multiple tasks and configurations.
cca_ros::PlanningResponse CcaRos::plan(const std::vector<cca_ros::PlanningRequest> &planning_requests)
{
    // Function output
    cca_ros::PlanningResponse planning_response;
    status_ = planning_response.status;
    *status_ = Status::PROCESSING;

    // Validate input
    try
    {
        if (planning_requests.size() == 1)
            this->validate_input_(planning_requests.front().task_description);
        else
            this->validate_input_(planning_requests);
    }
    catch (const std::invalid_argument &e)
    {
        RCLCPP_ERROR(node_logger_, "Error in input validation: %s", e.what());
        *status_ = Status::FAILED;
        return cca_ros::PlanningResponse();
    }

    const bool includes_gripper_trajectory = 
        !std::isnan(planning_requests.front().task_description.goal.gripper);

    cc_affordance_planner::PlannerResult &plannerResultFinal = planning_response.result;
    plannerResultFinal.planning_time = std::chrono::microseconds{0};

    // Separate trajectories for stitching
    std::vector<trajectory_msgs::msg::JointTrajectory> robot_trajectories;
    std::vector<trajectory_msgs::msg::JointTrajectory> gripper_trajectories;
    std::vector<trajectory_msgs::msg::JointTrajectory> combined_trajectories;

    cca_ros::GoalMsg goal_msg;

    // Iterate through all planner configurations
    std::vector<cca_ros::PlanningRequest> reqs = planning_requests;
    size_t i = 0;
    for (auto &planning_request : reqs)
    {
        const cc_affordance_planner::PlannerConfig &planner_config = planning_request.planner_config;
        cc_affordance_planner::TaskDescription task_description = planning_request.task_description;
        Eigen::VectorXd robot_start_config = planning_request.start_state.robot;
        double gripper_start_config = planning_request.start_state.gripper;

        // Get joint states if start configuration is empty
        if (robot_start_config.size() == 0)
        {
            try
            {
                const auto state = this->read_joint_states_();
                robot_start_config = state.robot;
            }
            catch (const std::runtime_error &e)
            {
                RCLCPP_ERROR(node_logger_, "Robot start config not available: %s", e.what());
                *status_ = Status::FAILED;
                return cca_ros::PlanningResponse();
            }
        }

        if (includes_gripper_trajectory && std::isnan(gripper_start_config))
        {
            try
            {
                const auto state = this->read_joint_states_();
                gripper_start_config = state.gripper;
            }
            catch (const std::runtime_error &e)
            {
                RCLCPP_ERROR(node_logger_, "Gripper start config not available: %s", e.what());
                *status_ = Status::FAILED;
                return cca_ros::PlanningResponse();
            }
        }

        // ----------------------------------------------------------------------------
        // If asked to preserve EE/tool orientation, compute planning requests to do that
        // ----------------------------------------------------------------------------
        if (task_description.ee_orientation_constraint == cc_affordance_planner::EeOrientationConstraint::PRESERVE)
        {
            // Discretize the screw path
            // Compute forward kinematics to tool
            const Eigen::Matrix4d fk = affordance_util::FKinSpace(M_, robot_slist_, robot_start_config);
            const std::vector<Eigen::Matrix4d> se3_screw_path =
                affordance_util::compute_se3_screw_trajectory(task_description.affordance_info,
                                                              task_description.goal.affordance,
                                                              task_description.trajectory_density,
                                                              fk);

            // Generate task descriptions from se3 screw trajectory
            bool preserve_orientation = true;
            const auto task_descriptions =
                cc_affordance_planner::get_se3_screw_tasks(se3_screw_path, preserve_orientation);

            // Build a vector of planning requests
            std::vector<cca_ros::PlanningRequest> sub_requests;
            for (const auto &td : task_descriptions)
            {
                cca_ros::PlanningRequest sub_req = planning_request;
                sub_req.task_description = td;
                sub_requests.push_back(sub_req);
            }

            RCLCPP_INFO(node_logger_, "Calling CCA planner with PRESERVE-ORIENTATION subrequests");

            // Call the vector-based planner and treat the response as one logical task
            auto sub_response = this->plan(sub_requests);

            if (!sub_response.result.success)
            {
                RCLCPP_ERROR(node_logger_, "Failed to plan preserve-orientation subtasks for task %zu", i);
                *status_ = Status::FAILED;
                return cca_ros::PlanningResponse();
            }

            // Merge sub-results as if it were a single planning task
            plannerResultFinal.joint_trajectory.insert(plannerResultFinal.joint_trajectory.end(),
                                                       sub_response.result.joint_trajectory.begin(),
                                                       sub_response.result.joint_trajectory.end());
            plannerResultFinal.planning_time += sub_response.result.planning_time;

            // Update start config for next main task
            planning_request.start_state.robot =
                sub_response.result.joint_trajectory.back().head(robot_joint_names_.size());
            if (includes_gripper_trajectory)
                planning_request.start_state.gripper =
                    sub_response.result.joint_trajectory.back()[gripper_joint_names_.size()];

            ++i;
            continue;
        }

        // ----------------------------------------------------------------------------
        // Prepare robot description for planning
        // ----------------------------------------------------------------------------
        affordance_util::RobotDescription robot_description;
        robot_description.slist = robot_slist_;
        robot_description.M = M_;
        robot_description.joint_states = robot_start_config;
        robot_description.gripper_state = gripper_start_config;

        // ----------------------------------------------------------------------------
        // Create and run the planner interface
        // ----------------------------------------------------------------------------
        cc_affordance_planner::PlannerResult planner_result;
        try
        {
            cc_affordance_planner::CcAffordancePlannerInterface ccAffordancePlannerInterface(planner_config);
            planner_result = ccAffordancePlannerInterface.generate_joint_trajectory(robot_description, task_description);
        }
        catch (const std::invalid_argument &e)
        {
            RCLCPP_ERROR(node_logger_, "Planner returned exception: %s", e.what());
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }

        // ----------------------------------------------------------------------------
        // Handle planner result
        // ----------------------------------------------------------------------------
        if (!planner_result.success)
        {
            RCLCPP_WARN(node_logger_, "Planner did not find a solution for task %zu", i);
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }

        if (planner_result.trajectory_description == cc_affordance_planner::TrajectoryDescription::PARTIAL)
        {
            int diff = task_description.trajectory_density - static_cast<int>(planner_result.joint_trajectory.size());
            if (std::abs(diff) >= 3)
            {
                RCLCPP_ERROR(node_logger_, "Trajectory partial: likely reached affordance limit.");
                *status_ = Status::FAILED;
                return cca_ros::PlanningResponse();
            }
            RCLCPP_WARN(node_logger_, "Trajectory partial but acceptable (within 3 points).");
        }

        // ----------------------------------------------------------------------------
        // Append successful results
        // ----------------------------------------------------------------------------
        plannerResultFinal.joint_trajectory.insert(plannerResultFinal.joint_trajectory.end(),
                                                   planner_result.joint_trajectory.begin(),
                                                   planner_result.joint_trajectory.end());
        plannerResultFinal.planning_time += planner_result.planning_time;

        goal_msg = this->create_goal_msg_(planner_result.joint_trajectory, includes_gripper_trajectory, planning_request.time_step);
        robot_trajectories.push_back(goal_msg.robot.trajectory);
        gripper_trajectories.push_back(goal_msg.gripper.trajectory);
        combined_trajectories.push_back(goal_msg.robot_and_gripper.trajectory);

        // Update start state for next iteration
        planning_request.start_state.robot = planner_result.joint_trajectory.back().head(robot_joint_names_.size());
        if (includes_gripper_trajectory)
            planning_request.start_state.gripper =
                planner_result.joint_trajectory.back()[gripper_joint_names_.size()];

        ++i;
    }

    // ----------------------------------------------------------------------------
    // Stitch trajectories together, validate, visualize, and execute
    // ----------------------------------------------------------------------------
    goal_msg.robot.trajectory = this->stitch_trajectories_(robot_trajectories);
    goal_msg.gripper.trajectory = this->stitch_trajectories_(gripper_trajectories);
    goal_msg.robot_and_gripper.trajectory = this->stitch_trajectories_(combined_trajectories);

    const auto cartesian_trajectory = this->compute_cartesian_trajectory_(plannerResultFinal.joint_trajectory);

    auto response = this->validate_and_visualize_(goal_msg.robot, cartesian_trajectory,
                                                  planning_requests.back().task_description);

    if (!response->success)
    {
        RCLCPP_ERROR(node_logger_, "%s validation failed. Trajectory likely violates constraints.",
                     viz_ss_name_.c_str());
        *status_ = Status::FAILED;
        return cca_ros::PlanningResponse();
    }

    plannerResultFinal.planning_time += std::cast<std::chrono::microseconds>(response->validation_time_usecs);

    if (planning_requests.front().execute_trajectory &&
        !(this->execute_(goal_msg, includes_gripper_trajectory)))
    {
        RCLCPP_ERROR(node_logger_, "Trajectory execution failed. See robot server for details.");
        *status_ = Status::FAILED;
        return cca_ros::PlanningResponse();
    }

    *status_ = Status::SUCCEEDED;
    return planning_response;
}


void CcaRos::initialize_action_clients_()
{
    // If robot and gripper execution server is available, that's all we need.
    if (!robot_and_gripper_traj_execution_as_name_.empty())
    {
        robot_and_gripper_traj_execution_client_ =
            rclcpp_action::create_client<FollowJointTrajectory>(this, robot_and_gripper_traj_execution_as_name_);
        unified_executor_available_ = true;
        return;
    }

    // Else initialize robot client
    robot_traj_execution_client_ =
        rclcpp_action::create_client<FollowJointTrajectory>(this, robot_traj_execution_as_name_);

    // Initialize gripper client in addition to the robot client if that is available
    if (!gripper_traj_execution_as_name_.empty())
    {
        // Only initialize if the gripper as name is provided
        gripper_traj_execution_client_ =
            rclcpp_action::create_client<FollowJointTrajectory>(this, gripper_traj_execution_as_name_);
    }
}
// Helper function to validate input
void CcaRos::validate_input_(const cc_affordance_planner::TaskDescription &task_description)
{
    if (!std::isnan(task_description.goal.gripper) && gripper_traj_execution_as_name_.empty() &&
        !unified_executor_available_)
    {
        throw std::invalid_argument("Task description: `goal.gripper` is specified, but `cca_gripper_as` or "
                                    "`cca_robot_and_gripper_as` parameters are"
                                    " not set up in the `cca_<robot>_ros_setup.yaml` file. Need one of them to be able "
                                    "to execute gripper trajectories");
    }
}

// Helper function to validate input
void CcaRos::validate_input_(const std::vector<cca_ros::PlanningRequest> reqs)
{

    // Ensure gripper goals are consistent across all tasks
    bool first_gripper_goal_status = !std::isnan(reqs.front().task_description.goal.gripper);
    this->validate_input_(reqs.front().task_description); // At the moment, we're just validating gripper info so, just
                                                      // check the first one.

    for (const auto &req: reqs)
    {
        bool gripper_goal_status = !std::isnan(req.task_description.goal.gripper);

        if (gripper_goal_status != first_gripper_goal_status)
        { // Check for logical inequivalence
            throw std::invalid_argument(
                "Task description: Inconsistent gripper goal across tasks. If one task considers the gripper goal, "
                "then all tasks must have the gripper goal set.");
        }
    }
}

// Helper function to get the full path to the robot description file.
std::string CcaRos::get_cc_affordance_robot_description_(const std::string &robot_name, const std::string &type)
{
    const std::string package_name = "cca_" + robot_name;
    const std::string rel_dir = "/config/";
    std::string filename;
    if (type=="yaml"){
        filename = package_name + "_description.yaml";
    }
    else if (type=="urdf"){
        filename = package_name + "_urdf.yaml";
    }
    return ros_cpp_util::get_filepath_inside_pkg(package_name, rel_dir, filename);
}

// Callback for joint_states topic.
void CcaRos::joint_states_cb_(const JointState::SharedPtr msg)
{
    robot_joint_states_ = ros_cpp_util::get_ordered_joint_states(msg, robot_joint_names_);
    gripper_joint_states_ = ros_cpp_util::get_ordered_joint_states(msg, gripper_joint_names_);
}

// Retrieve robot joint states at the start of the affordance.
KinematicState CcaRos::read_joint_states_()
{
    robot_joint_states_.positions.conservativeResize(robot_joint_names_.size());
    gripper_joint_states_.positions.conservativeResize(gripper_joint_names_.size());
    robot_joint_states_.positions.setConstant(std::numeric_limits<double>::quiet_NaN());
    gripper_joint_states_.positions.setConstant(std::numeric_limits<double>::quiet_NaN());

    auto start_time = this->now();
    rclcpp::Rate loop_rate(10); // 10 Hz loop rate
    const auto timeout = std::chrono::seconds(5);

    while (rclcpp::ok())
    {
        // Check joint states for NaN values
        if (!robot_joint_states_.positions.hasNaN() && !gripper_joint_states_.positions.hasNaN())
        {
            break;
        }

        // Check for timeout
        if ((this->now() - start_time) > rclcpp::Duration(timeout))
        {
            throw std::runtime_error("Failed to read robot or gripper joint states within timeout.");
        }

        // Allow for callback processing and sleep
        loop_rate.sleep();
    }

    return KinematicState{robot_joint_states_.positions, gripper_joint_states_.positions[0]};
}

std::vector<geometry_msgs::msg::Pose> CcaRos::compute_cartesian_trajectory_(
    const std::vector<Eigen::VectorXd> &trajectory)
{
    std::vector<geometry_msgs::msg::Pose> cartesian_trajectory;
    cartesian_trajectory.reserve(trajectory.size()); // Corrected typo

    for (const auto &point : trajectory)
    {
        // Compute FK
        Eigen::Matrix4d fk = affordance_util::FKinSpace(M_, robot_slist_, point.head(robot_joint_names_.size()));

        // Fill out the Pose msg
        Eigen::Quaterniond fk_quat(fk.block<3, 3>(0, 0)); // Extract rotation as quaternion
        geometry_msgs::msg::Pose pose;
        pose.position.x = fk(0, 3);
        pose.position.y = fk(1, 3);
        pose.position.z = fk(2, 3);

        // Corrected quaternion assignments
        pose.orientation.w = fk_quat.w();
        pose.orientation.x = fk_quat.x();
        pose.orientation.y = fk_quat.y();
        pose.orientation.z = fk_quat.z();

        // Store in the cartesian trajectory
        cartesian_trajectory.push_back(pose); // Corrected typo
    }
    return cartesian_trajectory;
}

// Function to create goal messages for robot and optionally for gripper
CcaRos::GoalMsg CcaRos::create_goal_msg_(
    const std::vector<Eigen::VectorXd> &trajectory, bool includes_gripper_trajectory, const TrajectoryTimeStep& time_step)
{
    // Initialize goal messages
    cca_ros::GoalMsg goal_msg;

    // Always create the robot goal message
    goal_msg.robot = ros_cpp_util::follow_joint_trajectory_msg_builder(
        trajectory, Eigen::VectorXd::Zero(robot_joint_names_.size()), robot_joint_names_, time_step.robot);

    if (includes_gripper_trajectory)
    {
        // Check if unified executor is available for combined trajectory
        if (unified_executor_available_)
        {
            // Combine robot and gripper joint names
            std::vector<std::string> robot_and_gripper_joint_names;
            robot_and_gripper_joint_names.reserve(robot_joint_names_.size() + gripper_joint_names_.size());
            robot_and_gripper_joint_names.insert(robot_and_gripper_joint_names.end(), robot_joint_names_.begin(),
                                                 robot_joint_names_.end());
            robot_and_gripper_joint_names.insert(robot_and_gripper_joint_names.end(), gripper_joint_names_.begin(),
                                                 gripper_joint_names_.end());

            // Build goal message for combined robot and gripper trajectory
            goal_msg.robot_and_gripper = ros_cpp_util::follow_joint_trajectory_msg_builder(
                trajectory, Eigen::VectorXd::Zero(robot_and_gripper_joint_names.size()), robot_and_gripper_joint_names,
                time_step.robot_and_gripper);
        }
        else
        {
            // Extract gripper trajectory from the full trajectory
            std::vector<Eigen::VectorXd> gripper_trajectory;
            gripper_trajectory.reserve(trajectory.size());

            for (const auto &point : trajectory)
            {
                Eigen::VectorXd gripper_point(1);
                gripper_point[0] = point[robot_joint_names_.size()];
                gripper_trajectory.push_back(gripper_point);
            }

            // Build goal message for gripper trajectory
            goal_msg.gripper = ros_cpp_util::follow_joint_trajectory_msg_builder(
                gripper_trajectory, Eigen::VectorXd::Zero(1), gripper_joint_names_, time_step.gripper);
        }
    }
    return goal_msg;
}

// Validates and visualizes a given trajectory
cca_ros_msgs::srv::CcaRosViz::Response::SharedPtr CcaRos::validate_and_visualize_(const FollowJointTrajectoryGoal &goal, const std::vector<geometry_msgs::msg::Pose>& cartesian_trajectory, const cc_affordance_planner::TaskDescription& task_description){

    // Extract affordance info
    const Eigen::Vector3d& w_aff = task_description.affordance_info.axis;
    const Eigen::Vector3d& q_aff = task_description.affordance_info.location;
    
    // Create visualization request
    auto viz_serv_req = std::make_shared<CcaRosViz::Request>();
    viz_serv_req->joint_traj = goal.trajectory;
    viz_serv_req->cartesian_traj = cartesian_trajectory;
    viz_serv_req->aff_screw_axis = {w_aff[0], w_aff[1], w_aff[2]};
    viz_serv_req->aff_location = {q_aff[0], q_aff[1], q_aff[2]};
    viz_serv_req->ref_frame = ref_frame_;

    // For APPROACH motion, fill out the affordance reference pose
    if (task_description.motion_type == cc_affordance_planner::MotionType::APPROACH)
    {
        // Position
        viz_serv_req->aff_ref_pose.position.x = task_description.goal.grasp_pose(0, 3);
        viz_serv_req->aff_ref_pose.position.y = task_description.goal.grasp_pose(1, 3);
        viz_serv_req->aff_ref_pose.position.z = task_description.goal.grasp_pose(2, 3);

        // Orientation
        Eigen::Quaterniond aff_ref_pose_quat(task_description.goal.grasp_pose.block<3, 3>(0, 0));
        aff_ref_pose_quat.normalize(); // Ensures it's a valid unit quaternion
        viz_serv_req->aff_ref_pose.orientation.w = aff_ref_pose_quat.w();
        viz_serv_req->aff_ref_pose.orientation.x = aff_ref_pose_quat.x();
        viz_serv_req->aff_ref_pose.orientation.y = aff_ref_pose_quat.y();
        viz_serv_req->aff_ref_pose.orientation.z = aff_ref_pose_quat.z();
    }
    else
    {
        // provide default sentinel values
        viz_serv_req->aff_ref_pose.position.x = 0;
        viz_serv_req->aff_ref_pose.position.y = 0;
        viz_serv_req->aff_ref_pose.position.z = 0;
        viz_serv_req->aff_ref_pose.orientation.w = 1;
        viz_serv_req->aff_ref_pose.orientation.x = 0;
        viz_serv_req->aff_ref_pose.orientation.y = 0;
        viz_serv_req->aff_ref_pose.orientation.z = 0;
    }

    // Wait for visualization service
    while (!viz_client_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node_logger_, "Interrupted while waiting for %s service. Exiting.", viz_ss_name_.c_str());
            *status_ = Status::FAILED;
	     auto response = std::make_shared<cca_ros_msgs::srv::CcaRosViz::Response>();
	     response->success = false;
	     return response;
        }
        RCLCPP_INFO(node_logger_, " %s service not available, waiting again...", viz_ss_name_.c_str());
    }

    // Send the request
    auto result_future = viz_client_->async_send_request(viz_serv_req);
    RCLCPP_INFO(node_logger_, "Sent trajectory visualization request to %s service", viz_ss_name_.c_str());
    auto response = result_future.get();
    return response;

}

bool CcaRos::execute_(const cca_ros::GoalMsg& goal_msg, bool includes_gripper_trajectory){

    // Setup goal options for sending trajectory goals
    auto robot_send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    robot_send_goal_options.goal_response_callback =
        std::bind(&CcaRos::robot_traj_execution_goal_response_callback_, this, std::placeholders::_1);
    robot_send_goal_options.result_callback =
        std::bind(&CcaRos::robot_traj_execution_result_callback_, this, std::placeholders::_1);
    
    // Check if both robot and gripper trajectories should be included
    if (includes_gripper_trajectory)
    {
        if (unified_executor_available_)
        {
            // Set result status and execute unified trajectory
            robot_result_status_ = status_;
            return this->send_execution_goal_(robot_and_gripper_traj_execution_client_, robot_send_goal_options,
                                       robot_and_gripper_traj_execution_as_name_, goal_msg.robot_and_gripper,
                                       unified_gh_future_);
        }
        else
        {
            // Setup goal options for gripper trajectory
            auto gripper_send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
            gripper_send_goal_options.goal_response_callback =
                std::bind(&CcaRos::gripper_traj_execution_goal_response_callback_, this, std::placeholders::_1);
            gripper_send_goal_options.result_callback =
                std::bind(&CcaRos::gripper_traj_execution_result_callback_, this, std::placeholders::_1);
    
            // Start a thread to check result status
            result_status_thread_ = std::jthread(&CcaRos::check_robot_and_gripper_result_status_, this);
    
            // Execute trajectories for both robot and gripper
            return (this->send_execution_goal_(robot_traj_execution_client_, robot_send_goal_options,
                                        robot_traj_execution_as_name_, goal_msg.robot, robot_gh_future_)) &&
                   (this->send_execution_goal_(gripper_traj_execution_client_, gripper_send_goal_options,
                                        gripper_traj_execution_as_name_, goal_msg.gripper, gripper_gh_future_));
        }
    }
    else
    {
        // Set result status and execute trajectory for robot only
        robot_result_status_ = status_;
        return this->send_execution_goal_(robot_traj_execution_client_, robot_send_goal_options,
                                   robot_traj_execution_as_name_, goal_msg.robot, robot_gh_future_);
    }
    
}

// Executes the planned trajectory.
bool CcaRos::send_execution_goal_(rclcpp_action::Client<FollowJointTrajectory>::SharedPtr &traj_execution_client,
                                 rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_goal_options,
                                 const std::string &traj_execution_as_name, const FollowJointTrajectoryGoal &goal,
                                 std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> &goal_handle_future)
{
    // Before execution, ensure current state does not deviate much from trajectory start state
    try
    {
        const KinematicState current_state = read_joint_states_();
        const Eigen::VectorXd goal_state =
            Eigen::VectorXd::Map(goal.trajectory.points[0].positions.data(), current_state.robot.size());
        const double tolerance = 1 * 1e-1; // Declare tolerance as double

        // Compare goal state and current state within the tolerance
        if ((goal_state - current_state.robot).cwiseAbs().maxCoeff() > tolerance)
        {
            RCLCPP_ERROR(node_logger_, "Refusing to execute trajectory due to the current robot state being "
                                       "significantly different from the trajectory start state.");

            // Format current state
            std::stringstream current_state_stream;
            current_state_stream << current_state.robot.transpose().format(
                Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));

            // Format goal state
            std::stringstream goal_state_stream;
            goal_state_stream << goal_state.transpose().format(
                Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]"));

            // Log the states
            RCLCPP_ERROR(node_logger_, "Current State: %s", current_state_stream.str().c_str());
            RCLCPP_ERROR(node_logger_, "Trajectory Start State: %s", goal_state_stream.str().c_str());

            *status_ = Status::FAILED;
            return false;
        }
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_ERROR(node_logger_, "Robot state unavailable during pre-execution check: %s", e.what());
        *status_ = Status::FAILED;
        return false;
    }

    // Wait for the action server to be ready
    if (!traj_execution_client->wait_for_action_server())
    {
        RCLCPP_ERROR(node_logger_, " %s action server not available after waiting", traj_execution_as_name.c_str());
        *status_ = Status::FAILED;
        return false;
    }

    RCLCPP_INFO(node_logger_, "Sending goal to %s action server", traj_execution_as_name.c_str());
    goal_handle_future = traj_execution_client->async_send_goal(goal, send_goal_options);
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
    // Start statuses as processing
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

void CcaRos::cancel_execution()
{
    // Struct to hold the cancel future and corresponding action server name
    struct CancelRequest
    {
        std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>> cancel_future;
        std::string action_server_name;

        CancelRequest(std::shared_future<std::shared_ptr<action_msgs::srv::CancelGoal_Response>> cancel_future_,
                      const std::string &action_server_name_)
            : cancel_future(std::move(cancel_future_)), action_server_name(action_server_name_)
        {
        }
    };

    // Create a vector to store cancel requests
    std::vector<CancelRequest> cancel_requests;

    // Check and send cancellation request for unified goal (robot and gripper)
    if (unified_gh_future_.valid())
    {
        cancel_requests.push_back(
            CancelRequest(robot_and_gripper_traj_execution_client_->async_cancel_goal(unified_gh_future_.get()),
                          robot_and_gripper_traj_execution_as_name_));
        RCLCPP_INFO(node_logger_, "Attempting to cancel %s goal", robot_and_gripper_traj_execution_as_name_.c_str());
    }

    // Check and send cancellation request for robot goal
    if (robot_gh_future_.valid())
    {
        cancel_requests.push_back(CancelRequest(robot_traj_execution_client_->async_cancel_goal(robot_gh_future_.get()),
                                                robot_traj_execution_as_name_));
        RCLCPP_INFO(node_logger_, "Attempting to cancel %s goal", robot_traj_execution_as_name_.c_str());
    }

    // Check and send cancellation request for gripper goal
    if (gripper_gh_future_.valid())
    {
        cancel_requests.push_back(
            CancelRequest(gripper_traj_execution_client_->async_cancel_goal(gripper_gh_future_.get()),
                          gripper_traj_execution_as_name_));
        RCLCPP_INFO(node_logger_, "Attempting to cancel %s goal", gripper_traj_execution_as_name_.c_str());
    }

    // Now check the responses from all cancellation requests
    for (const auto &cancel_request : cancel_requests)
    {
        // Wait for the cancellation response and check the return code
        auto cancel_response = cancel_request.cancel_future.get();
        const std::string &action_server_name = cancel_request.action_server_name; // Action server name

        if (cancel_response->return_code == action_msgs::srv::CancelGoal_Response::ERROR_NONE)
        {
            RCLCPP_INFO(node_logger_, "%s goal canceled successfully", action_server_name.c_str());
        }
        else
        {
            RCLCPP_ERROR(node_logger_, "Failed to cancel goal for action server %s", action_server_name.c_str());
        }
    }
}

trajectory_msgs::JointTrajectory CcaRos::stitch_trajectories_(const std::vector<trajectory_msgs::JointTrajectory>& trajectories)
{
    trajectory_msgs::JointTrajectory result;
    if (trajectories.empty())
        return result;

    result.joint_names = trajectories[0].joint_names;
    ros::Duration time_offset(0.0);

    for (size_t i = 0; i < trajectories.size(); ++i)
    {
        const auto& traj = trajectories[i];

        // Optional: ensure joint names match
	if (traj.joint_names != result.joint_names) {
            throw std::runtime_error("Joint names mismatch in trajectory " + std::to_string(i));
        }

        // Adjust the timing of each point
        for (const auto& p : traj.points)
        {
            trajectory_msgs::JointTrajectoryPoint new_pt = p;
            new_pt.time_from_start += time_offset;
            result.points.push_back(new_pt);
        }

        // Update total time offset to the last point of this trajectory
        if (!traj.points.empty())
            time_offset = result.points.back().time_from_start;
    }

    return result;
}

} // namespace cca_ros
