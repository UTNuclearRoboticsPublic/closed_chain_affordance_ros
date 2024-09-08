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
#include <cca_ros_viz_msgs/srv/cca_ros_viz.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::chrono_literals;
class CcaRosVizServer : public rclcpp::Node
{
  public:
    explicit CcaRosVizServer(const rclcpp::NodeOptions &options)
        : Node("cca_ros_viz_server_node", options),
          node_logger_(this->get_logger()),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {

        // Create and advertise planning and visualization service
        srv_ = this->create_service<cca_ros_viz_msgs::srv::CcaRosViz>(
            "/cca_ros_viz_server", std::bind(&CcaRosVizServer::cca_ros_viz_server_callback_, this,
                                             std::placeholders::_1, std::placeholders::_2));

        // Create a publisher to publish EE trajectory
        ee_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ee_trajectory", 10);

        // Initialize the publisher to show moveit planned path
        moveit_planned_path_pub_ =
            this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
        RCLCPP_INFO_STREAM(node_logger_, "/cca_ros_viz service server is active");
    }

    ~CcaRosVizServer() { rclcpp::shutdown(); }
    void start_and_spin_executor()
    {
        // Add node to executor and start executor thread
        node_handle_ = this->shared_from_this();
        executor_->add_node(this->shared_from_this());
        std::thread([this]() { executor_->spin(); }).detach();

        // Initialize planning parameters
        robot_model_loader::RobotModelLoaderPtr robot_model_loader =
            std::make_shared<robot_model_loader::RobotModelLoader>(node_handle_);
        psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_handle_, robot_model_loader);
        moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();
        /* robot_state = std::make_shared<moveit::core::RobotState>(robot_model); */
        planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
            robot_model, node_handle_, "planning_plugin", "request_adapters");
        robot_state_ = std::make_shared<moveit::core::RobotState>(
            planning_scene_monitor::LockedPlanningSceneRO(psm_)
                ->getCurrentState()); // planning scene is locked while reading robot
        joint_model_group_ = robot_model->getJointModelGroup("arm");
        psm_->startSceneMonitor();
        psm_->startWorldGeometryMonitor(); // listens to world geometry, collision objects and (optionally) octomap
                                           // changes
        psm_->startStateMonitor(
            "/spot_driver/joint_states"); // listens to joint state updates and attached collision object changes
        rviz_visual_tools_.reset(
            new rviz_visual_tools::RvizVisualTools("base_link", "/rviz_visual_tools", node_handle_));
        /* dynamic_cast<rclcpp::Node *>(this))); */
        rviz_visual_tools_->loadMarkerPub();
        rviz_visual_tools_->enableBatchPublishing();
        moveit_visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_handle_, "arm0_base_link",
                                                                                        "cca_ros_viz", psm_);
    }

  private:
    // Variables
    rclcpp::Node::SharedPtr node_handle_; // handle for the node

    rclcpp::Logger node_logger_; // logger associated with the node
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
        executor_;                // executor needed for MoveIt robot state checking
    std::thread executor_thread_; // thread
    rclcpp::Service<cca_ros_viz_msgs::srv::CcaRosViz>::SharedPtr srv_; // joint traj plan and visualization service
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ee_traj_pub_; // publisher for EE trajectory
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
        moveit_planned_path_pub_; // publisher to show moveit planned path

    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;
    moveit::core::RobotStatePtr robot_state_;
    moveit::core::JointModelGroup *joint_model_group_;
    rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;
    moveit_visual_tools::MoveItVisualToolsPtr moveit_visual_tools_;

    // Methods
    void cca_ros_viz_server_callback_(const std::shared_ptr<cca_ros_viz_msgs::srv::CcaRosViz::Request> serv_req,
                                      std::shared_ptr<cca_ros_viz_msgs::srv::CcaRosViz::Response> serv_res)
    {

        bool has_sub = rviz_visual_tools_->waitForMarkerSub(0.25);
        if (!has_sub)
            RCLCPP_INFO(node_logger_, "/rviz_visual_tools does not have a subscriber. Visualizations may be lost. "
                                      "Ensure /rviz_visual_tools is "
                                      "specified as topic under MarkerArray in Rviz. ");
        // Clear messages
        rviz_visual_tools_->deleteAllMarkers();

        // Create motion plan request
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;
        req.group_name = serv_req->planning_group;

        // Moveit messages to visualize planned path and hold planning pipeline response
        moveit_msgs::msg::DisplayTrajectory display_trajectory;
        moveit_msgs::msg::MotionPlanResponse response;

        RCLCPP_INFO(node_logger_, "Planning and visualizing the trajectory");

        if (!(serv_req->aff_screw_axis).empty()) // If affordance screw is specified, draw it
        {
            // Display the affordance screw axis
            // Note the affordance screw axis in the service request is wrt to the reference frame and here we are
            // publishing wrt to the planning frame
            Eigen::Isometry3d T_p_r = robot_state_->getGlobalLinkTransform(
                serv_req->ref_frame); // planning frame (which is usually the base frame from the URDF) to the reference
                                      // frame.

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
            aff_screw_pose = T_p_r * aff_screw_pose;

            // Publish
            rviz_visual_tools_->publishArrow(aff_screw_pose, rviz_visual_tools::CYAN, rviz_visual_tools::LARGE);
            rviz_visual_tools_->trigger();
        }

        for (const auto &point : serv_req->joint_traj.points)
        {
            // copy the joint trajectory point to a std::vector<double> type
            std::vector<double> planning_end_state(point.positions.begin(), point.positions.end());

            // Set the planning goal state to that trajectory point
            moveit::core::RobotState goal_state(*robot_state_);
            goal_state.setJointGroupPositions(joint_model_group_, planning_end_state);
            moveit_msgs::msg::Constraints joint_goal =
                kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group_);
            req.goal_constraints.clear();
            req.goal_constraints.push_back(joint_goal);

            // Acquire read-only lock on the planning scene before planning and generate plan
            {
                planning_scene_monitor::LockedPlanningSceneRO lscene(psm_);
                planning_pipeline_->generatePlan(lscene, req, res);
            }

            // Check if planning was successful, exit if not
            if (res.error_code_.val != res.error_code_.SUCCESS)
            {
                RCLCPP_ERROR(node_logger_, "Could not compute plan successfully");
                return;
            }

            // Visualize the plan
            res.getMessage(response); // copy the generated plan to the moveit_msgs response variable
            display_trajectory.trajectory_start =
                response.trajectory_start;                                // update the start of the display trajectory
            display_trajectory.trajectory.push_back(response.trajectory); // update the whole trajectory
            moveit_planned_path_pub_->publish(display_trajectory);        // publish the trajectory
            moveit_visual_tools_->publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group_);

            // Set the start state in the planning scene to the final state of the last plan
            robot_state_->setJointGroupPositions(joint_model_group_,
                                                 response.trajectory.joint_trajectory.points.back().positions);
            moveit::core::robotStateToRobotStateMsg(*robot_state_, req.start_state);

            // Compute forward kinematics to the EE and store the position and orientation in the marker variable
            Eigen::Isometry3d ee_htm = robot_state_->getGlobalLinkTransform(serv_req->tool_frame);

            // Publish the EE frame
            rviz_visual_tools_->publishAxis(ee_htm);
            rviz_visual_tools_->trigger();
        }

        RCLCPP_INFO(node_logger_, "Successfully planned and visualized the trajectory");
        serv_res->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcaRosVizServer>(node_options);
    node->start_and_spin_executor();

    while (rclcpp::ok())
    {
    }
    return 0;
}
