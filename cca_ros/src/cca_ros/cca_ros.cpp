#include "cca_ros/cca_ros.hpp"
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>

namespace cca_ros
{

// Constructor for CcaRos, initializes the node and sets up required parameters and clients.
CcaRos::CcaRos(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options),
      node_logger_(this->get_logger()),   // Logger for the node
      viz_ss_name_("/cca_ros_viz_server") // Name of the service to validate and visualize result
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


cca_ros::PlanningResponse CcaRos::plan(const cca_ros::PlanningRequest &planning_request) {
    // Delegate to the multi-request planner with a single-element vector
    return plan(std::vector<cca_ros::PlanningRequest>{planning_request});
}

cca_ros::PlanningResponse CcaRos::plan(const std::vector<cca_ros::PlanningRequest> &planning_requests) {

    // Declare function output and set status
    cca_ros::PlanningResponse planning_response;
    status_ = planning_response.status;
    *status_ = Status::PROCESSING;

    // Validate input based on whether we have single or multiple requests
    try {

        this->validate_input_(planning_requests);

    } catch (const std::invalid_argument &e) {

        RCLCPP_ERROR(node_logger_, "Error in input validation: %s", e.what());
        *status_ = Status::FAILED;
        return cca_ros::PlanningResponse();

    }

    // Determine if gripper trajectory is included
    const bool includes_gripper_trajectory = !std::isnan(planning_requests.front().task_description.goal.gripper);

    // Read start state if not provided
    cca_ros::KinematicState current_state = planning_requests.front().start_state;
    
    if (current_state.robot.size() == 0) {
        try {
            KinematicState state = read_joint_states_();
            current_state.robot = state.robot;
        } catch (const std::runtime_error &e) {
            RCLCPP_ERROR(node_logger_, "Robot start config not available: %s", e.what());
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }
    }
    
    if (includes_gripper_trajectory && std::isnan(current_state.gripper)) {
        try {
            KinematicState state = read_joint_states_();
            current_state.gripper = state.gripper;
        } catch (const std::runtime_error &e) {
            RCLCPP_ERROR(node_logger_, "Gripper start config not available: %s", e.what());
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }
    }

    // Define some helper structs
    // Struct to collect trajectory messages for stitching
    struct TrajMsg {
        std::vector<trajectory_msgs::msg::JointTrajectory> robot;
        std::vector<trajectory_msgs::msg::JointTrajectory> gripper;
        std::vector<trajectory_msgs::msg::JointTrajectory> robot_and_gripper;
    } traj_msg;

    // Struct to hold indexed planning requests
    struct IndexedPlanningRequest {
        size_t org_index; // Original index
        cca_ros::PlanningRequest request;
    };

    // Prepare robot description for planning -- We'll fill in states in the following loop
    affordance_util::RobotDescription robot_description;
    robot_description.slist = robot_slist_;
    robot_description.M = M_;

    // Make a working copy of requests that we can modify (fill in start state or expand EE orientation preservation tasks)
    std::vector<IndexedPlanningRequest> working_requests;
    for (size_t i =0; i<planning_requests.size(); ++i){
        IndexedPlanningRequest req;
	req.request = planning_requests[i];
        req.org_index = i;
        working_requests.push_back(req);
	}

    // Initialize aggregated planner result
    cc_affordance_planner::PlannerResult& planner_result_final = planning_response.result;
    planner_result_final.planning_time = std::chrono::microseconds{0};

    // Some helper variables
    // See if we have a single planning request
    const bool single_planning_request = planning_requests.size() == 1;
    bool is_partial = true; // To track if solved trajectories are partial

    // Process each planning request
    for (size_t task_idx = 0; task_idx < working_requests.size(); ++task_idx) {
        const size_t org_task_idx = working_requests[task_idx].org_index;
        const std::string index_log = single_planning_request ? "" : " for task " + std::to_string(org_task_idx);
        auto& request = working_requests[task_idx].request;
        auto& task_description = request.task_description;
        auto& start_state = request.start_state;
        
        // Set start state from current state (first task or previous task end state)
        start_state = current_state;

        // Lookup affordance info if requested
        if (task_description.affordance_info.from.method==affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME) {
            try {
                // Lookup transform from ref_frame_ to the lookup frame
                const geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform(
                        ref_frame_, 
                        task_description.affordance_info.from.frame_name,
                        tf2::TimePointZero);  // Get latest available transform
                
                // Convert to Eigen type so we could do some math
		const Eigen::Isometry3d T_ref_to_lookup_frame = tf2::transformToEigen(transform_stamped.transform);

		// Apply requested transform -- We now have the transform from the reference frame to the desired affordance frame
                const Eigen::Isometry3d T_ref_to_aff = T_ref_to_lookup_frame * Eigen::Isometry3d(task_description.affordance_info.from.post_transform);

                // Extract translation from the transform
	        task_description.affordance_info.location = T_ref_to_aff.translation();	

		// Compute the requested affordance axis in reference frame
                if (!task_description.affordance_info.from.axis_in_final_pose.hasNaN()){
		    task_description.affordance_info.axis = T_ref_to_aff.linear() * task_description.affordance_info.from.axis_in_final_pose;
		}

                // Set affordance_info specification method to PROVIDED since we have everything now
                task_description.affordance_info.from.method = affordance_util::PoseSpecificationMethod::PROVIDED; 
                
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(node_logger_, 
                    "Could not lookup transform from %s to %s to fill in affordance info%s: %s", 
                    ref_frame_.c_str(),
                    task_description.affordance_info.from.frame_name.c_str(),
		    index_log.c_str(),
                    ex.what());
                *status_ = Status::FAILED;
                return cca_ros::PlanningResponse();
            }
        }

        // Lookup canonical frame info if requested
        if (task_description.motion_type==cc_affordance_planner::MotionType::APPROACH && task_description.canonical_pose_from.method==affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME) {
            try {
                // Lookup transform from ref_frame_ to the lookup frame
                const geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform(
                        ref_frame_, 
                        task_description.canonical_pose_from.frame_name,
                        tf2::TimePointZero);  // Get latest available transform
                
                // Convert to Eigen type so we could do some math
		const Eigen::Isometry3d T_ref_to_lookup_frame = tf2::transformToEigen(transform_stamped.transform);

		// Apply requested transform -- We now have the transform from the reference frame to the desired canonical frame
                const Eigen::Isometry3d T_ref_to_can = T_ref_to_lookup_frame * Eigen::Isometry3d(task_description.canonical_pose_from.post_transform);

                // Extract translation from the transform
	        task_description.goal.canonical_pose = T_ref_to_can.matrix();	

                // Set canonical_pose specification method to PROVIDED since we have everything now
                task_description.canonical_pose_from.method = affordance_util::PoseSpecificationMethod::PROVIDED; 
                
            } catch (const tf2::TransformException &ex) {
                RCLCPP_ERROR(node_logger_, 
                    "Could not lookup transform from %s to %s to fill in canonical pose%s: %s", 
                    ref_frame_.c_str(),
                    task_description.canonical_pose_from.frame_name.c_str(),
		    index_log.c_str(),
                    ex.what());
                *status_ = Status::FAILED;
                return cca_ros::PlanningResponse();
            }
        }

        // Check if this task requires EE orientation preservation
        if (task_description.ee_orientation_constraint == 
            cc_affordance_planner::EeOrientationConstraint::PRESERVE) {
            
            // Compute forward kinematics and discretize screw path
            const Eigen::Matrix4d fk = affordance_util::FKinSpace(M_, robot_slist_, start_state.robot);
            const std::vector<Eigen::Matrix4d> se3_screw_path = 
                affordance_util::compute_se3_screw_trajectory(
                    task_description.affordance_info, 
                    task_description.goal.affordance, 
                    task_description.trajectory_density, 
                    fk);
            
            // Generate subtask descriptions from SE(3) trajectory
            const bool preserve_orientation = true;
            const std::vector<cc_affordance_planner::TaskDescription> subtask_descriptions = 
                cc_affordance_planner::get_se3_screw_tasks(se3_screw_path, preserve_orientation);
            
            // Create planning requests for each subtask
            // std::vector<cca_ros::PlanningRequest> subtask_requests;
            std::vector<IndexedPlanningRequest> subtask_requests;
            subtask_requests.reserve(subtask_descriptions.size());
            
            for (size_t i = 0; i < subtask_descriptions.size(); ++i) {
                IndexedPlanningRequest subtask_request;
                subtask_request.request.planner_config = request.planner_config;
                subtask_request.request.task_description = subtask_descriptions[i];
                subtask_request.request.time_step = request.time_step;
                subtask_request.request.execute_trajectory = request.execute_trajectory;
                // Only first subtask uses current start state; others will be chained
                subtask_request.request.start_state = (i == 0) ? start_state : cca_ros::KinematicState();
		subtask_request.org_index = org_task_idx;
                subtask_requests.push_back(subtask_request);
            }
            
            // Replace current task with subtasks in the working_requests vector
            working_requests.erase(working_requests.begin() + task_idx);
            working_requests.insert(working_requests.begin() + task_idx, 
                                   subtask_requests.begin(), subtask_requests.end());

            // Skip the stale task and continue on from the next iteration	
            --task_idx; 
            continue;    
        }

	// Add joint states to robot description for planning
        robot_description.joint_states = start_state.robot;
        robot_description.gripper_state = start_state.gripper;

        // Create and run the planner interface
        cc_affordance_planner::CcAffordancePlannerInterface planner(request.planner_config);
        cc_affordance_planner::PlannerResult task_result;
        
        try {
            task_result = planner.generate_joint_trajectory(robot_description, task_description);
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(node_logger_, "Planner returned exception%s: %s", index_log.c_str(), e.what());
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }

        // Check if planning succeeded
        if (!task_result.success) {
            RCLCPP_WARN(node_logger_, "Planner did not find a solution%s", index_log.c_str());
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }

        // Fail for partial trajectories unless it is just a single planning request
	is_partial = task_result.trajectory_description == cc_affordance_planner::TrajectoryDescription::PARTIAL;
        if (is_partial && !single_planning_request) {
                const double affordance_limit = 
                    std::copysign(task_result.joint_trajectory.back().tail(1)(0), task_description.goal.affordance);
                RCLCPP_ERROR(node_logger_,
                    "Partial solution%s. Could be due to affordance reaching limit at %f. Try "
                    "readjusting the task to this limit.", index_log.c_str(), affordance_limit);
                *status_ = Status::FAILED;
                return cca_ros::PlanningResponse();
        }

        // Aggregate results -- append joint trajectories and accumulate planning time
        planner_result_final.joint_trajectory.insert(
            planner_result_final.joint_trajectory.end(),
            task_result.joint_trajectory.begin(),
            task_result.joint_trajectory.end());
        planner_result_final.planning_time += task_result.planning_time;

        // Everything else the same as the last task result
	planner_result_final.includes_gripper_trajectory = task_result.includes_gripper_trajectory;
	planner_result_final.trajectory_description = task_result.trajectory_description;
	planner_result_final.success = task_result.success;
	planner_result_final.update_method = task_result.update_method;
	planner_result_final.update_trail = task_result.update_trail;

        // Create goal message for this task and collect trajectories -- we'll stitch them together later for validation, visualization, and execution
        cca_ros::GoalMsg goal_msg = 
            this->create_goal_msg_(task_result.joint_trajectory, includes_gripper_trajectory, request.time_step);
        traj_msg.robot.push_back(goal_msg.robot.trajectory);
        traj_msg.gripper.push_back(goal_msg.gripper.trajectory);
        traj_msg.robot_and_gripper.push_back(goal_msg.robot_and_gripper.trajectory);

        // Update current state from this task's end state
        current_state.robot = task_result.joint_trajectory.back().head(robot_joint_names_.size());
        if (includes_gripper_trajectory) {
            current_state.gripper = task_result.joint_trajectory.back()[robot_joint_names_.size()];
        }
    }

    // Stitch all trajectories together into final goal message
    cca_ros::GoalMsg final_goal_msg;
    final_goal_msg.robot.trajectory = ros_cpp_util::stitch_trajectories(traj_msg.robot);
    final_goal_msg.gripper.trajectory = ros_cpp_util::stitch_trajectories(traj_msg.gripper);
    final_goal_msg.robot_and_gripper.trajectory = ros_cpp_util::stitch_trajectories(traj_msg.robot_and_gripper);

    // For single original task, check if aggregated trajectory is partial and allow small deviation
    if (single_planning_request && is_partial) {
        const int traj_size_difference = 
            planning_requests.front().task_description.trajectory_density - 
            static_cast<int>(planner_result_final.joint_trajectory.size());
        const double affordance_limit = 
            std::copysign(planner_result_final.joint_trajectory.back().tail(1)(0), 
                         planning_requests.front().task_description.goal.affordance);

        if (std::abs(traj_size_difference) < 3) {
            RCLCPP_WARN(node_logger_,
                "Trajectory description: PARTIAL with %d points less than FULL. "
                "Could be due to affordance reaching limit at %f. Try "
                "readjusting the task to this limit. Will allow execution of trajectory, but do so with caution.",
                traj_size_difference, affordance_limit);
        } else {
            RCLCPP_ERROR(node_logger_,
                "Trajectory description: PARTIAL. Could be due to affordance reaching limit at %f. Try "
                "readjusting the task to this limit.", affordance_limit);
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }
    }

    // Compute cartesian trajectory for the tool
    const std::vector<geometry_msgs::msg::Pose> cartesian_trajectory = 
        this->compute_cartesian_trajectory_(planner_result_final.joint_trajectory);
    
    // Validate and visualize the complete trajectory (use last task description)
    // TODO: Visualize all task descriptions instead of just the last one
    const auto& final_task_description = working_requests.back().request.task_description;
    auto validation_response = this->validate_and_visualize_(
        final_goal_msg.robot, cartesian_trajectory, final_task_description);
    
    if (!validation_response->success) {
        RCLCPP_ERROR(node_logger_, 
            "%s validation service failed. Trajectory likely violates self-collision or joint limit constraints. "
            "Check server for more info.", viz_ss_name_.c_str());
        *status_ = Status::FAILED;
        return cca_ros::PlanningResponse();
    }

    RCLCPP_INFO(node_logger_, " %s validation service succeeded", viz_ss_name_.c_str());
    planner_result_final.planning_time += 
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::microseconds(validation_response->validation_time_usecs));

    // Execute trajectory if requested (check first request for execute flag)
    if (planning_requests.front().execute_trajectory) {
        if (!this->execute_(final_goal_msg, includes_gripper_trajectory)) {
            RCLCPP_ERROR(node_logger_, 
                "Validated trajectory execution failed. See robot server side for more info.");
            *status_ = Status::FAILED;
            return cca_ros::PlanningResponse();
        }
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
void CcaRos::validate_input_(const std::vector<cca_ros::PlanningRequest>& reqs)
{
    const bool single_planning_request = reqs.size() == 1;
    const bool gripper_goal_specified = !std::isnan(reqs.front().task_description.goal.gripper);
    
    // Gripper executor availability check
    if (gripper_goal_specified && gripper_traj_execution_as_name_.empty() && !unified_executor_available_)
    {
        throw std::invalid_argument("Task description: `goal.gripper` is specified, but `cca_gripper_as` or "
                                    "`cca_robot_and_gripper_as` parameters are"
                                    " not set up in the `cca_<robot>_ros_setup.yaml` file. Need one of them to be able "
                                    "to execute gripper trajectories");
    }
    
    for (size_t task_index = 0; task_index < reqs.size(); ++task_index)
    {
        const auto &req = reqs[task_index];
        const std::string index_log = single_planning_request ? "" : "Task " + std::to_string(task_index) + ": ";
        
        // Ensure gripper goals are consistent (compare against first task)
        if (task_index > 0) {
            bool gripper_goal_status = !std::isnan(req.task_description.goal.gripper);
            if (gripper_goal_status != gripper_goal_specified) {
                throw std::invalid_argument(
                    index_log + "Inconsistent gripper goal specification. All tasks must either specify a gripper goal or leave it unspecified");
            }
        }
        
        // Validate frame_name is supplied if asked to lookup screw_info from frame name
        if ((req.task_description.affordance_info.from.method == affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME) && 
            (req.task_description.affordance_info.from.frame_name.empty())) {
            throw std::invalid_argument(
                index_log + "task_description.affordance_info: from.method FROM_FRAME_NAME requires from.frame_name, but is empty");
        }

        // Ensure screw axis is provided when looking up affordance info using the "from" member
        if (req.task_description.affordance_info.from.axis_in_final_pose.hasNaN() && 
            req.task_description.affordance_info.axis.hasNaN() && req.task_description.affordance_info.screw.hasNaN()) {
            throw std::invalid_argument(
                index_log + "task_description.affordance_info: Either from.axis_in_final_pose or affordance_info.axis or affordance_info.screw must be provided");
        }

        // Validate canonical frame name is supplied if asked to lookup canonical pose from frame name
        if ((req.task_description.canonical_pose_from.method == affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME) && 
            (req.task_description.canonical_pose_from.frame_name.empty())) {
            throw std::invalid_argument(
                index_log + "task_description.canonical_pose_from: method FROM_FRAME_NAME requires frame_name, but is empty");
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
cca_ros::GoalMsg CcaRos::create_goal_msg_(
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
        viz_serv_req->aff_ref_pose.position.x = task_description.goal.canonical_pose(0, 3);
        viz_serv_req->aff_ref_pose.position.y = task_description.goal.canonical_pose(1, 3);
        viz_serv_req->aff_ref_pose.position.z = task_description.goal.canonical_pose(2, 3);

        // Orientation
        Eigen::Quaterniond aff_ref_pose_quat(task_description.goal.canonical_pose.block<3, 3>(0, 0));
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

} // namespace cca_ros
