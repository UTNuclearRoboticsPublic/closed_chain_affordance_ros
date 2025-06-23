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
#include <rclcpp/rclcpp.hpp>
#include <cca_ros_msgs/srv/cca_ros_viz.hpp>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::chrono_literals;
class CcaRosVizServer : public rclcpp::Node
{
  public:
    explicit CcaRosVizServer(const rclcpp::NodeOptions &options)
        : Node("cca_ros_viz_server_node", options), node_logger_(this->get_logger())
    {

        // Extract parameters
        // robot_description and robot_description_semantic automatically extracted during runtime
        planning_group_ = this->get_parameter("planning_group").as_string();
        rviz_fixed_frame_ = this->get_parameter("rviz_fixed_frame").as_string();
        joint_states_topic_ = this->get_parameter("joint_states_topic").as_string();

        // Create and advertise planning and visualization service
        srv_ = this->create_service<cca_ros_msgs::srv::CcaRosViz>(
            "/cca_ros_viz_server", std::bind(&CcaRosVizServer::cca_ros_viz_server_callback_, this,
                                             std::placeholders::_1, std::placeholders::_2));

        // Initialize the publisher to show moveit planned path
        moveit_planned_path_pub_ =
            this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
        RCLCPP_INFO_STREAM(node_logger_, "/cca_ros_viz service server is active");
    }

    ~CcaRosVizServer()
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
        moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(
            planning_scene_monitor::LockedPlanningSceneRO(psm_)
                ->getCurrentState()); // planning scene is locked while reading robot
        joint_model_group_ = robot_model->getJointModelGroup(planning_group_);
        psm_->startSceneMonitor();
        psm_->startWorldGeometryMonitor(); // listens to world geometry, collision objects and (optionally) octomap
                                           // changes
        psm_->startStateMonitor(
            joint_states_topic_); // listens to joint state updates and attached collision object changes

        // Make the planning‐scene service available for diffs and publish the scene -- needed to reflect joint states correctly
        psm_->providePlanningSceneService();
        psm_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

        rviz_visual_tools_.reset(
            new rviz_visual_tools::RvizVisualTools(rviz_fixed_frame_, "/cca_ee_cartesian_trajectory", node_handle));
        rviz_visual_tools_->loadMarkerPub();
        rviz_visual_tools_->enableBatchPublishing();
    }

  private:
    // Variables
    rclcpp::Node::SharedPtr node_handle;
    std::thread spinner_thread_; // To spin the node in a separate thread

    rclcpp::Logger node_logger_;                                       // logger associated with the node
    rclcpp::Service<cca_ros_msgs::srv::CcaRosViz>::SharedPtr srv_; // joint traj plan and visualization service
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
        moveit_planned_path_pub_; // publisher to show moveit planned path

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    moveit::core::RobotStatePtr robot_state_;
    moveit::core::JointModelGroup *joint_model_group_;
    rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;

    std::string planning_group_;
    std::string rviz_fixed_frame_;
    std::string joint_states_topic_;

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


    void cca_ros_viz_server_callback_(const std::shared_ptr<cca_ros_msgs::srv::CcaRosViz::Request> serv_req,
                                      std::shared_ptr<cca_ros_msgs::srv::CcaRosViz::Response> serv_res)
    {

        serv_res->success = false;// start as false

        bool has_sub = rviz_visual_tools_->waitForMarkerSub(0.25);
        if (!has_sub)
            RCLCPP_INFO(node_logger_, "/rviz_visual_tools does not have a subscriber. Visualizations may be lost. "
                                      "Ensure /rviz_visual_tools is "
                                      "specified as topic under MarkerArray in Rviz. ");
        // Clear messages
        rviz_visual_tools_->deleteAllMarkers();


        RCLCPP_INFO(node_logger_, "Planning and visualizing the trajectory");

        // Capture T_w_r, the HTM from world frame, usually the root frame of the urdf to the service request reference
        // frame
        Eigen::Isometry3d T_w_r = robot_state_->getGlobalLinkTransform(serv_req->ref_frame);

        if (!(serv_req->aff_screw_axis).empty()) // If affordance screw is specified, draw it
        {

            // Rviz puts arrows along x-axis by default. So, get the quaternion representation of the affordance screw
            // axis wrt to the x-axis.
            Eigen::Quaterniond aff_screw_quat;
            aff_screw_quat.setFromTwoVectors(Eigen::Vector3d::UnitX(),
                                             Eigen::Vector3d((serv_req->aff_screw_axis).data()));

            // Fill out the pose
            Eigen::Isometry3d aff_screw_pose;
            aff_screw_pose.linear() = aff_screw_quat.toRotationMatrix();
            aff_screw_pose.translation() = Eigen::Vector3d((serv_req->aff_location).data());

            // Translate the pose to planning frame
            aff_screw_pose = T_w_r * aff_screw_pose;

            // If affordance ref frame is specified, draw it
            if (this->is_pose_specified(serv_req->aff_ref_pose))
            {
                Eigen::Isometry3d aff_ref_pose = this->transform_pose_to_world_frame(T_w_r, serv_req->aff_ref_pose);

                rviz_visual_tools_->publishAxis(aff_ref_pose, rviz_visual_tools::Scales::LARGE);
            }

            // Publish
            rviz_visual_tools_->publishArrow(aff_screw_pose, rviz_visual_tools::CYAN, rviz_visual_tools::LARGE);
            rviz_visual_tools_->trigger();
        }

	// (Re)order trajectory to match MoveIt planning group order
	trajectory_msgs::msg::JointTrajectory ordered_group_traj = reorder_trajectory_(serv_req->joint_traj, joint_model_group_->getVariableNames());

	long total_viol_check_duration = 0; // for joint limits and collision checking

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
		long point_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
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

		    RCLCPP_ERROR(node_logger_, "Generated trajectory violates constraints [%s] at point: [%s]",
		    	     violation_type.c_str(), oss.str().c_str());

		    // If self-collision occurs, print the contacts
		    if (self_collision_violation){
			    collision_detection::CollisionResult::ContactMap::const_iterator it;
			    for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
			    {
			      RCLCPP_ERROR(node_logger_, "Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
			    }
		    
		    }

		    return;

		}
            }
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
	// RCLCPP_INFO(node_logger_, "Total constraint violation checking time for trajectory: %ld microseconds", total_viol_check_duration); For experiment purposes
        serv_res->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcaRosVizServer>(node_options);
    node->initialize();

    while (rclcpp::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Sleep to avoid busy waiting
    }
    rclcpp::shutdown();
    return 0;
}
