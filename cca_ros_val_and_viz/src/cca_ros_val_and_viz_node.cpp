///////////////////////////////////////////////////////////////////////////////
//      Title     : cca_ros_viz_node.cpp
//      Project   : cca_ros_viz
//      Created   : Jan 2024
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
#include <fmt/core.h>
#include <iomanip>
#include <sstream>  
#include <string>  
#include <rclcpp/rclcpp.hpp>
#include <cca_ros_msgs/srv/cca_ros_val_and_viz.hpp>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Custom utility headers
#include <ros_cpp_util/ros_cpp_util.hpp>

using namespace std::chrono_literals;
class CcaRosValAndVizServer : public rclcpp::Node
{
  public:
    explicit CcaRosValAndVizServer(const rclcpp::NodeOptions &options)
        : Node("cca_ros_val_and_viz", options), node_logger_(this->get_logger()), val_and_viz_ss_name_("/cca_ros_val_and_viz") 
    {

        // Extract parameters
        // robot_description and robot_description_semantic automatically extracted during runtime
        rviz_fixed_frame_ = ros_cpp_util::get_required_str_param(this, "rviz_fixed_frame");
        joint_states_topic_ = ros_cpp_util::get_required_str_param(this, "joint_states_topic");

        // Create and advertise planning and visualization service
        srv_ = this->create_service<cca_ros_msgs::srv::CcaRosValAndViz>(
            val_and_viz_ss_name_, std::bind(&CcaRosValAndVizServer::cca_ros_viz_server_callback_, this,
                                             std::placeholders::_1, std::placeholders::_2));

        // Initialize the publisher to show moveit planned path
        moveit_planned_path_pub_ =
            this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
        RCLCPP_INFO_STREAM(node_logger_, val_and_viz_ss_name_ <<" service is active");
    }

    ~CcaRosValAndVizServer()
    {
        // Cleanup spinner thread
        if (spinner_thread_.joinable())
        {
            spinner_thread_.join();
        }
    }
    void initialize()
    {
        // Spin node in a separate thread so we can start reading robot state
        node_handle = this->shared_from_this();
        spinner_thread_ = std::thread([this]() { rclcpp::spin(node_handle); });

        // Initialize planning parameters
        robot_model_loader::RobotModelLoaderPtr robot_model_loader =
            std::make_shared<robot_model_loader::RobotModelLoader>(node_handle);
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_handle, robot_model_loader);
        robot_model_ = robot_model_loader->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(
            planning_scene_monitor::LockedPlanningSceneRO(psm_)
                ->getCurrentState()); // planning scene is locked while reading robot
        psm_->startSceneMonitor();
        psm_->startWorldGeometryMonitor(); // listens to world geometry, collision objects and (optionally) octomap
                                           // changes
        psm_->startStateMonitor(
            joint_states_topic_); // listens to joint state updates and attached collision object changes

        // Make the planning‐scene service available for diffs and publish the scene -- needed to reflect joint states correctly
        psm_->providePlanningSceneService();
        psm_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

        rviz_visual_tools_.reset(
            new rviz_visual_tools::RvizVisualTools(rviz_fixed_frame_, val_and_viz_ss_name_, node_handle));
        rviz_visual_tools_->loadMarkerPub(); 	    // Initialize publisher
        rviz_visual_tools_->setLifetime(0.0);       // Publish markers with zero timestamp to avoid future extrapolation
        rviz_visual_tools_->enableFrameLocking();   // Keep markers fixed in the RViz frame to bypass TF transforms
        rviz_visual_tools_->enableBatchPublishing();// Batch publishing for efficiency

    }

  private:
    // Variables
    rclcpp::Node::SharedPtr node_handle;
    std::thread spinner_thread_; // To spin the node in a separate thread

    rclcpp::Logger node_logger_;                                       // logger associated with the node
    rclcpp::Service<cca_ros_msgs::srv::CcaRosValAndViz>::SharedPtr srv_; // joint traj plan and visualization service
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
        moveit_planned_path_pub_; // publisher to show moveit planned path

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    moveit::core::RobotStatePtr robot_state_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::JointModelGroup *joint_model_group_;
    rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;

    std::string val_and_viz_ss_name_;
    std::string rviz_fixed_frame_;
    std::string joint_states_topic_;

    std::string get_joint_limit_violation_log_(const moveit::core::RobotState& state, const std::map<std::string, moveit::core::VariableBounds>& joint_limit_map) {
        const int JOINT_NAME_WIDTH = 30;
        const int VALUE_WIDTH = 15;
        const int LIMIT_WIDTH = 15;
        const int TOTAL_WIDTH = JOINT_NAME_WIDTH + VALUE_WIDTH + LIMIT_WIDTH + LIMIT_WIDTH;
        const int FLOAT_PRECISION = 4;
        
        std::stringstream error_msg;
        error_msg << std::fixed << std::setprecision(FLOAT_PRECISION);
        error_msg << "Offending joints:\n";
        error_msg << std::setw(JOINT_NAME_WIDTH) << std::left << "Joint name" 
                  << std::setw(VALUE_WIDTH) << "Value" 
                  << std::setw(LIMIT_WIDTH) << "Min Limit" 
                  << std::setw(LIMIT_WIDTH) << "Max Limit" << "\n";
        error_msg << std::string(TOTAL_WIDTH, '-') << "\n";
        
        for (const auto& [joint_name, bounds] : joint_limit_map) {
            const moveit::core::JointModel* joint_model = 
                state.getRobotModel()->getJointModel(joint_name);
            
            if (joint_model && !state.satisfiesBounds(joint_model)) {
                const double value = state.getVariablePosition(joint_name);
                
                error_msg << std::setw(JOINT_NAME_WIDTH) << std::left << joint_name
                          << std::setw(VALUE_WIDTH) << value
                          << std::setw(LIMIT_WIDTH) << bounds.min_position_
                          << std::setw(LIMIT_WIDTH) << bounds.max_position_ << "\n";
            }
        }
        
        return error_msg.str();
    }

    // Note, T_w_r is HTM from world frame, usually the root frame of the urdf to the service request reference frame
    Eigen::Isometry3d transform_pose_to_world_frame(const Eigen::Isometry3d &T_w_r,
                                                    const geometry_msgs::msg::Pose &pose)
    {
        // Convert the pose to an Eigen::Isometry3d type
        Eigen::Isometry3d eigen_pose;
        eigen_pose.linear() =
            Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
                .toRotationMatrix();
        eigen_pose.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);

        // Translate the pose to the planning frame
        return T_w_r * eigen_pose; // Return transformed pose
    }

    bool is_pose_specified(const geometry_msgs::msg::Pose &pose)
    {
        // Check if the pose has non-default position and orientation values
        if (pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 && pose.orientation.x == 0.0 &&
            pose.orientation.y == 0.0 && pose.orientation.z == 0.0 && pose.orientation.w == 1.0)
        {
            // Pose is default, likely not specified
            return false;
        }
        // Pose has been specified
        return true;
    }

    // Reorders the trajectory to match a given joint name order (e.g., for a planning group or the full robot)
    trajectory_msgs::msg::JointTrajectory reorder_trajectory_(
        const trajectory_msgs::msg::JointTrajectory &input_traj,
        const std::vector<std::string> &target_joint_names)
    {
        trajectory_msgs::msg::JointTrajectory ordered_traj;
        ordered_traj.header = input_traj.header;
        ordered_traj.joint_names = target_joint_names;
    
        // Build name → index map from input
        std::unordered_map<std::string, size_t> name_to_index;
        for (size_t i = 0; i < input_traj.joint_names.size(); ++i)
        {
    	name_to_index[input_traj.joint_names[i]] = i;
        }
    
        // Pull a fresh robot state
        moveit::core::RobotState fresh_state(*robot_state_);
        {
    	planning_scene_monitor::LockedPlanningSceneRO scene(psm_);
    	fresh_state = scene->getCurrentState();
        }
    
        // Reorder each point according to target_joint_names
        for (const auto &point : input_traj.points)
        {
    	trajectory_msgs::msg::JointTrajectoryPoint new_point;
    	new_point.time_from_start = point.time_from_start;
    	new_point.positions.resize(target_joint_names.size());
    
    	for (size_t i = 0; i < target_joint_names.size(); ++i)
    	{
    	    const auto &name = target_joint_names[i];
    	    auto it = name_to_index.find(name);
    	    if (it != name_to_index.end())
    	    {
    		new_point.positions[i] = point.positions[it->second];
    	    }
    	    else
    	    {
    		new_point.positions[i] = fresh_state.getVariablePosition(name);
    		// RCLCPP_ERROR(node_logger_, "Joint '%s' missing in trajectory point. Using current robot state.",
    		// 	     name.c_str());
    	    }
    	}
    
    	ordered_traj.points.push_back(std::move(new_point));
        }
    
        return ordered_traj;
    }


    void cca_ros_viz_server_callback_(const std::shared_ptr<cca_ros_msgs::srv::CcaRosValAndViz::Request> serv_req,
                                      std::shared_ptr<cca_ros_msgs::srv::CcaRosValAndViz::Response> serv_res)
    {

        serv_res->success = false;// start as false

        // Clear messages
        rviz_visual_tools_->deleteAllMarkers();

        RCLCPP_INFO(node_logger_, "Planning and visualizing the trajectory");

        // Capture T_w_r, the HTM from world frame, usually the root frame of the urdf to the service request reference
        // frame
        Eigen::Isometry3d T_w_r = robot_state_->getGlobalLinkTransform(serv_req->ref_frame);

        // Validate affordance info sizes
        if (serv_req->aff_screw_axes.size() != serv_req->aff_locations.size() ||
	    serv_req->aff_screw_axes.size() != serv_req->aff_ref_poses.size())
	{
	    RCLCPP_ERROR(node_logger_,
			 "Mismatch in the size of affordance screw axes, locations, and reference pose vectors");
	    return;
	}

        // Draw affordance screw axes and optionally, aff ref frames
        for (size_t task_idx = 0; task_idx < serv_req->aff_screw_axes.size(); ++task_idx){
            const auto aff_screw_axis = serv_req->aff_screw_axes.at(task_idx);
	    const auto aff_location = serv_req->aff_locations.at(task_idx);
            const auto aff_ref_pose_msg = serv_req->aff_ref_poses.at(task_idx);

            // Rviz puts arrows along x-axis by default. So, get the quaternion representation of the affordance screw
            // axis wrt to the x-axis.
            Eigen::Quaterniond aff_screw_quat;
            aff_screw_quat.setFromTwoVectors(Eigen::Vector3d::UnitX(),
                                             Eigen::Vector3d(aff_screw_axis.x, aff_screw_axis.y, aff_screw_axis.z));

            // Fill out the pose
            Eigen::Isometry3d aff_screw_pose;
            aff_screw_pose.linear() = aff_screw_quat.toRotationMatrix();
            aff_screw_pose.translation() = Eigen::Vector3d(aff_location.x, aff_location.y, aff_location.z);

            // Translate the pose to planning frame
            aff_screw_pose = T_w_r * aff_screw_pose;

            // If affordance ref frame is specified, draw it
            if (this->is_pose_specified(aff_ref_pose_msg))
            {
                Eigen::Isometry3d aff_ref_pose = this->transform_pose_to_world_frame(T_w_r, aff_ref_pose_msg);

                rviz_visual_tools_->publishAxis(aff_ref_pose, rviz_visual_tools::Scales::LARGE);
            }

            // Publish
            rviz_visual_tools_->publishArrow(aff_screw_pose, rviz_visual_tools::CYAN, rviz_visual_tools::LARGE);
            rviz_visual_tools_->trigger();
	}

        // Get the joint model group for the requested planning group
        joint_model_group_ = robot_model_->getJointModelGroup(serv_req->planning_group);

	// Capture joint names for the planning group
	std::vector<std::string> joint_names = joint_model_group_->getVariableNames();

        // Capture joint limits so we could log joint-limit violations later
        std::map<std::string, moveit::core::VariableBounds> joint_limit_map;
        for (const std::string& joint_name : joint_names) {
            joint_limit_map[joint_name] = 
                robot_model_->getVariableBounds(joint_name);
        }

	// (Re)order trajectory to match MoveIt planning group order
	trajectory_msgs::msg::JointTrajectory ordered_group_traj = reorder_trajectory_(serv_req->joint_traj, joint_names);

        std::chrono::microseconds total_viol_check_duration{0}; // for joint limits and collision checking

	size_t pt_index = 0;
        for (const auto &point : ordered_group_traj.points)
        {
            // Copy the joint trajectory point to a std::vector<double> type
            std::vector<double> planning_end_state(point.positions.begin(), point.positions.end());

            // Set the planning goal state to that trajectory point
            moveit::core::RobotState goal_state(*robot_state_);
            goal_state.setJointGroupPositions(joint_model_group_, planning_end_state);
            moveit_msgs::msg::Constraints joint_goal =
                kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_);

            // Acquire read-only lock on the planning scene before doing anything
            {
                planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);

		// Check for joint limit and self-collision violation
		auto start_time = std::chrono::high_resolution_clock::now(); // start time for this point in traj

		// Set up collision requests and results
		collision_detection::CollisionRequest collision_request;
		collision_request.contacts = true;
		collision_request.max_contacts = 1000;
		collision_detection::CollisionResult collision_result;
		collision_result.clear();

		// Check and store violation check
		bool joint_limit_violation = !goal_state.satisfiesBounds(joint_model_group_);
		psm_->getPlanningScene()->checkSelfCollision(collision_request, collision_result, goal_state);
		bool self_collision_violation = collision_result.collision;

		// Capture how long it took to check for violations
		auto end_time = std::chrono::high_resolution_clock::now(); // stop time for this point in traj
		auto point_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
		total_viol_check_duration += point_duration;

		// Log violation
		if (joint_limit_violation || self_collision_violation) {

		    std::string violation_type =
                    (joint_limit_violation && self_collision_violation) ? "Joint Limit Violation & Self-Collision" :
                    joint_limit_violation ? "Joint Limit Violation" : "Self-Collision";

		    
		    const double* goal_positions = goal_state.getVariablePositions();
		    size_t num_joints = goal_state.getVariableCount();  // Get the number of joint values

		    std::ostringstream oss;
		    for (size_t k = 0; k < num_joints; ++k) {
		        if (k > 0) oss << ", ";
		        oss << goal_positions[k];
		    }

		    RCLCPP_ERROR(node_logger_, "Generated trajectory violates constraints [%s] at point[%zu]: [%s]",
		    	     violation_type.c_str(), pt_index, oss.str().c_str());

		    // If self-collision occurs, print the contacts
		    if (self_collision_violation){
			    collision_detection::CollisionResult::ContactMap::const_iterator it;
			    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
			    {
			      RCLCPP_ERROR(node_logger_, "Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
			    }
		    
		    }

		    // If joint-limit violation occurs, print the offending joints and info
		    if (joint_limit_violation){	
                            const std::string jl_err_log = get_joint_limit_violation_log_(goal_state, joint_limit_map);  
			    RCLCPP_ERROR(node_logger_, jl_err_log.c_str());
		    }

		    return;

		}
            }
	    ++pt_index;
        }

	// Since no joint‐limit or self‐collision violation, now visualize the trajectory
	// Transform the trajectory to the full robot trajectory for visualization, i.e. by adding the current state of the unplanned joints
	trajectory_msgs::msg::JointTrajectory ordered_robot_traj = reorder_trajectory_(serv_req->joint_traj, robot_state_->getVariableNames());

	moveit_msgs::msg::DisplayTrajectory display_trajectory;

	// Set start state 
	display_trajectory.trajectory_start.joint_state.name     = ordered_robot_traj.joint_names;
	display_trajectory.trajectory_start.joint_state.position = ordered_robot_traj.points.front().positions;

	// Fill out the trajectory
	auto &robot_traj = display_trajectory.trajectory.emplace_back();
	robot_traj.joint_trajectory = ordered_robot_traj;

	// Publish the joint trajectory
	moveit_planned_path_pub_->publish(display_trajectory);

        // Publish the tool trajectory
	for (const auto& pose : serv_req->cartesian_traj)
	{
	    rviz_visual_tools_->publishAxis(this->transform_pose_to_world_frame(T_w_r, pose));
	}
	rviz_visual_tools_->trigger();  // only once after batching

        RCLCPP_INFO(node_logger_, "Successfully visualized requested joint trajectory");
        serv_res->success = true;
        serv_res->validation_time_usecs = total_viol_check_duration.count(); // in microseconds
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    auto node = std::make_shared<CcaRosValAndVizServer>(node_options);
    node->initialize();

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to avoid busy waiting
    }
    rclcpp::shutdown();
    return 0;
}
