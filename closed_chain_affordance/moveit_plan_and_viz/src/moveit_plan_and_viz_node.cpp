///////////////////////////////////////////////////////////////////////////////
//      Title     : moveit_plan_and_viz_node.cpp
//      Project   : moveit_plan_and_viz
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
#include <moveit_plan_and_viz/srv/move_it_plan_and_viz.hpp>
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
class MoveItPlanAndVizServer : public rclcpp::Node
{
  public:
    MoveItPlanAndVizServer(const rclcpp::NodeOptions &options)
        : Node("moveit_plan_and_viz_server_node", options),
          node_logger_(this->get_logger()),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {

        // Create and advertise planning and visualization service
        srv_ = this->create_service<moveit_plan_and_viz::srv::MoveItPlanAndViz>(
            "/moveit_plan_and_viz_server", std::bind(&MoveItPlanAndVizServer::moveit_plan_and_viz_server_callback_,
                                                     this, std::placeholders::_1, std::placeholders::_2));

        // Create a publisher to publish EE trajectory
        ee_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ee_trajectory", 10);

        // Initialize the publisher to show moveit planned path
        moveit_planned_path_pub_ =
            this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
        RCLCPP_INFO_STREAM(node_logger_, "/moveit_plan_and_viz service server is active");
    }

    ~MoveItPlanAndVizServer() { rclcpp::shutdown(); }
    void start_and_spin_executor()
    {
        // Add node to executor and start executor thread
        node_handle_ = this->shared_from_this();
        executor_->add_node(this->shared_from_this());
        std::thread([this]() { executor_->spin(); }).detach();

        // Initialize planning parameters
        robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_handle_);
        psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_handle_, robot_model_loader);
        robot_model = robot_model_loader->getModel();
        /* robot_state = std::make_shared<moveit::core::RobotState>(robot_model); */
        planning_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
            robot_model, node_handle_, "planning_plugin", "request_adapters");
        robot_state = std::make_shared<moveit::core::RobotState>(
            planning_scene_monitor::LockedPlanningSceneRO(psm)
                ->getCurrentState()); // planning scene is locked while reading robot
        joint_model_group = robot_model->getJointModelGroup("arm");
        psm->startSceneMonitor();
        psm->startWorldGeometryMonitor(); // listens to world geometry, collision objects and (optionally) octomap
                                          // changes
        psm->startStateMonitor(
            "/spot_driver/joint_states"); // listens to joint state updates and attached collision object changes
    }

  private:
    // Variables
    rclcpp::Node::SharedPtr node_handle_; // handle for the node

    rclcpp::Logger node_logger_; // logger associated with the node
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
        executor_;                // executor needed for MoveIt robot state checking
    std::thread executor_thread_; // thread
    rclcpp::Service<moveit_plan_and_viz::srv::MoveItPlanAndViz>::SharedPtr
        srv_; // joint traj plan and visualization service
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ee_traj_pub_; // publisher for EE trajectory
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
        moveit_planned_path_pub_; // publisher to show moveit planned path

    robot_model_loader::RobotModelLoaderPtr robot_model_loader;
    planning_scene_monitor::PlanningSceneMonitorPtr psm;
    moveit::core::RobotModelPtr robot_model;
    planning_pipeline::PlanningPipelinePtr planning_pipeline;
    moveit::core::RobotStatePtr robot_state;
    moveit::core::JointModelGroup *joint_model_group;

    // Methods
    void moveit_plan_and_viz_server_callback_(
        const std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Request> serv_req,
        std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Response> serv_res)
    {
        // Basic housekeeping for planning, need robot model and planning scene monitor, which observes changes in
        // planning scene
        /* robot_model_loader::RobotModelLoaderPtr robot_model_loader( */
        /*     new robot_model_loader::RobotModelLoader(node_handle_, serv_req->robot_description)); */

        /* planning_scene_monitor::PlanningSceneMonitorPtr psm( */
        /*     new planning_scene_monitor::PlanningSceneMonitor(node_handle_, robot_model_loader)); */

        // Start listening to changes in the planning scene
        /* psm->startSceneMonitor(); */
        /* psm->startWorldGeometryMonitor(); // listens to world geometry, collision objects and (optionally) octomap */
        // changes
        /* psm->startStateMonitor();         // listens to joint state updates and attached collision object changes */

        // Get robot model
        /* moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel(); */

        // Create planning pipeline object using the robot model
        /* planning_pipeline::PlanningPipelinePtr planning_pipeline( */
        /*     new planning_pipeline::PlanningPipeline(robot_model, node_handle_, "planning_plugin",
         * "request_adapters")); */

        // Visualization using MoveIt Visual Tools
        namespace rvt = rviz_visual_tools;
        rvt::RvizVisualToolsPtr visual_tools_;
        moveit_visual_tools::MoveItVisualTools visual_tools(node_handle_, serv_req->ref_frame, "moveit_plan_and_viz",
                                                            psm);

        visual_tools_.reset(
            new rvt::RvizVisualTools("base_link", "/rviz_visual_tools", dynamic_cast<rclcpp::Node *>(this)));
        // create publisher before waiting
        visual_tools_->loadMarkerPub();
        bool has_sub = visual_tools_->waitForMarkerSub(1.0);
        if (!has_sub)
            RCLCPP_INFO(get_logger(), "/rviz_visual_tools does not have a subscriber after 10s. "
                                      "Visualizations may be lost");

        // Create pose
        Eigen::Isometry3d pose;
        pose = Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
        pose.translation() = Eigen::Vector3d(0.1, 0.1, 0.1);          // translate x,y,z

        // Clear messages
        visual_tools_->deleteAllMarkers();
        visual_tools_->enableBatchPublishing();
        // Publish arrow vector of pose
        visual_tools_->publishArrow(pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

        // Don't forget to trigger the publisher!
        visual_tools_->trigger();
        // Create motion plan request
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;
        req.group_name = serv_req->planning_group;

        // Moveit messages to visualize planned path and hold planning pipeline response
        moveit_msgs::msg::DisplayTrajectory display_trajectory;
        moveit_msgs::msg::MotionPlanResponse response;

        // To ensure we plan from the current state of the robot, get the most up-to-date state
        /* moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState( */
        /*     planning_scene_monitor::LockedPlanningSceneRO(psm) */
        /*         ->getCurrentState())); // planning scene is locked while reading robot */
        // state to ensure no changes happen during the reading
        moveit::core::robotStateToRobotStateMsg(
            *robot_state, req.start_state); // update the planning request start state to be this current state

        // Create a JointModelGroup to keep track of the group we are planning for
        /* const moveit::core::JointModelGroup *joint_model_group = */
        /* robot_state->getJointModelGroup(serv_req->planning_group); */

        // Extract the joint trajectory from the service request and plan and visualize for every subsequent points in
        // the trajectory
        const size_t lof_joint_traj = serv_req->joint_traj.points.size(); // length of the trajectory

        // EE trajectory markers
        visualization_msgs::msg::MarkerArray ee_traj;               // entire trajectory
        visualization_msgs::msg::Marker ee_traj_point;              // a point within trajectory
        ee_traj_point.header.frame_id = serv_req->rviz_fixed_frame; // frame of reference for markers
        ee_traj_point.ns = "ee_origin";
        ee_traj_point.type = visualization_msgs::msg::Marker::SPHERE;
        ee_traj_point.action = visualization_msgs::msg::Marker::ADD;
        ee_traj_point.scale.x = 0.01;
        ee_traj_point.scale.y = 0.01;
        ee_traj_point.scale.z = 0.01;
        ee_traj_point.color.a = 1.0;
        ee_traj_point.color.r = 0.0;
        ee_traj_point.color.g = 1.0;
        ee_traj_point.color.b = 0.0;

        visualization_msgs::msg::Marker ee_traj_arrow_x;
        ee_traj_arrow_x.header.frame_id = serv_req->rviz_fixed_frame;
        ee_traj_arrow_x.ns = "ee_x_axis";
        ee_traj_arrow_x.type = visualization_msgs::msg::Marker::ARROW;
        ee_traj_arrow_x.action = visualization_msgs::msg::Marker::ADD;
        ee_traj_arrow_x.scale.x = 0.1;
        ee_traj_arrow_x.scale.y = 0.01;
        ee_traj_arrow_x.scale.z = 0.01;
        ee_traj_arrow_x.color.a = 1.0;
        ee_traj_arrow_x.color.r = 1.0;
        ee_traj_arrow_x.color.g = 0.0;
        ee_traj_arrow_x.color.b = 0.0;

        visualization_msgs::msg::Marker ee_traj_arrow_y;
        ee_traj_arrow_y.header.frame_id = serv_req->rviz_fixed_frame;
        ee_traj_arrow_y.ns = "ee_y_axis";
        ee_traj_arrow_y.type = visualization_msgs::msg::Marker::ARROW;
        ee_traj_arrow_y.action = visualization_msgs::msg::Marker::ADD;
        ee_traj_arrow_y.scale.x = 0.1;
        ee_traj_arrow_y.scale.y = 0.01;
        ee_traj_arrow_y.scale.z = 0.01;
        ee_traj_arrow_y.color.a = 1.0;
        ee_traj_arrow_y.color.r = 0.0;
        ee_traj_arrow_y.color.g = 1.0;
        ee_traj_arrow_y.color.b = 0.0;

        visualization_msgs::msg::Marker ee_traj_arrow_z;
        ee_traj_arrow_z.header.frame_id = serv_req->rviz_fixed_frame;
        ee_traj_arrow_z.ns = "ee_z_axis";
        ee_traj_arrow_z.type = visualization_msgs::msg::Marker::ARROW;
        ee_traj_arrow_z.action = visualization_msgs::msg::Marker::ADD;
        ee_traj_arrow_z.scale.x = 0.1;
        ee_traj_arrow_z.scale.y = 0.01;
        ee_traj_arrow_z.scale.z = 0.01;
        ee_traj_arrow_z.color.a = 1.0;
        ee_traj_arrow_z.color.r = 0.0;
        ee_traj_arrow_z.color.g = 0.0;
        ee_traj_arrow_z.color.b = 1.0;
        for (size_t j = 0; j < lof_joint_traj; j++)
        {
            // Extract the next joint trajectory point from the service request for planning
            trajectory_msgs::msg::JointTrajectoryPoint &point = serv_req->joint_traj.points[j];
            std::vector<double> planning_end_state(point.positions.begin(), point.positions.end());

            // Set the planning goal state to that trajectory point
            moveit::core::RobotState goal_state(*robot_state);
            goal_state.setJointGroupPositions(joint_model_group, planning_end_state);
            moveit_msgs::msg::Constraints joint_goal =
                kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
            req.goal_constraints.clear();
            req.goal_constraints.push_back(joint_goal);

            // Acquire read-only lock on the planning scene before planning and generate plan
            {
                planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
                planning_pipeline->generatePlan(lscene, req, res);
            }

            // Check if planning was successful, exit if not
            if (res.error_code_.val != res.error_code_.SUCCESS)
            {
                RCLCPP_ERROR(node_logger_, "Could not compute plan successfully");
                return;
            }

            // Visualize the plan
            RCLCPP_INFO(node_logger_, "Visualizing the trajectory");
            res.getMessage(response); // copy the generated plan to the moveit_msgs response variable
            display_trajectory.trajectory_start =
                response.trajectory_start;                                // update the start of the display trajectory
            display_trajectory.trajectory.push_back(response.trajectory); // update the whole trajectory
            moveit_planned_path_pub_->publish(display_trajectory);        // publish the trajectory
            visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);

            // Set the start state in the planning scene to the final state of the last plan
            robot_state->setJointGroupPositions(joint_model_group,
                                                response.trajectory.joint_trajectory.points.back().positions);
            moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

            // Compute forward kinematics to the EE and store the position and orientation in the marker variable
            Eigen::Isometry3d ee_htm = robot_state->getGlobalLinkTransform(serv_req->tool_frame);
            geometry_msgs::msg::Pose waypoint;
            // Set the translation (position) values
            waypoint.position.x = ee_htm.translation().x();
            waypoint.position.y = ee_htm.translation().y();
            waypoint.position.z = ee_htm.translation().z();
            // Set the rotation (orientation) values (quaternion)
            Eigen::Quaterniond ee_quat(ee_htm.rotation());
            waypoint.orientation.x = ee_quat.x();
            waypoint.orientation.y = ee_quat.y();
            waypoint.orientation.z = ee_quat.z();
            waypoint.orientation.w = ee_quat.w();

            // Update EE trajectory origin marker characteristics
            ee_traj_point.header.stamp = this->get_clock()->now();
            ee_traj_point.id = j;
            ee_traj_point.pose = waypoint; // Assuming waypoints contain Pose information

            /* // Update EE trajectory x-axis marker characteristics */
            /* Eigen::Vector3d axis_direction; */
            /* std::cout << "Here is ee_htm.linear\n" << ee_htm.linear() << std::endl; */
            /* Eigen::Vector3d x_axis = ee_htm * Eigen::Vector4d(1, 0, 0, 1).head<3>(); */
            /* Eigen::Vector3d y_axis = ee_htm * Eigen::Vector4d(0, 1, 0, 1).head<3>(); */
            /* Eigen::Vector3d z_axis = ee_htm * Eigen::Vector4d(0, 0, 1, 1).head<3>(); */
            /* std::cout << "Here is ee_htm.linear.col(0)\n" << x_axis << std::endl; */
            /* std::cout << "Here is ee_htm.linear.col(1)\n" << y_axis << std::endl; */
            /* std::cout << "Here is ee_htm.linear.col(2)\n" << z_axis << std::endl; */
            /* /1* Eigen::Quaterniond ee_orientation(waypoint.orientation.w, waypoint.orientation.x,
             * waypoint.orientation.y, */
            /*  *1/ */
            /* /1*                                   waypoint.orientation.z); *1/ */
            ee_traj_arrow_x.header.stamp = this->get_clock()->now();
            ee_traj_arrow_x.id = j * 2;
            /* /1* ee_traj_arrow_x.pose.position.x = ee_htm.translation().x(); *1/ */
            /* /1* ee_traj_arrow_x.pose.position.y = ee_htm.translation().y(); *1/ */
            /* /1* ee_traj_arrow_x.pose.position.z = ee_htm.translation().z(); *1/ */
            /* /1* Eigen::Vector3d x_axis(1, 0, 0); *1/ */
            /* /1* axis_direction = (ee_orientation * x_axis).normalized(); *1/ */
            /* /1* ee_traj_arrow_x.pose.orientation.w = 1.0; *1/ */
            /* /1* ee_traj_arrow_x.pose.orientation.x = axis_direction.x(); *1/ */
            /* /1* ee_traj_arrow_x.pose.orientation.y = axis_direction.y(); *1/ */
            /* /1* ee_traj_arrow_x.pose.orientation.z = axis_direction.z(); *1/ */
            /* std::cout << "waypoint.position.x" << waypoint.position.x << std::endl; */
            /* std::cout << "waypoint.position.y" << waypoint.position.y << std::endl; */
            /* ee_traj_arrow_x.points.resize(2); */
            /* std::cout << "waypoint.position.z" << waypoint.position.z << std::endl; */
            /* ee_traj_arrow_x.points[0].x = waypoint.position.x; */
            /* ee_traj_arrow_x.points[0].y = waypoint.position.y; */
            /* ee_traj_arrow_x.points[0].z = waypoint.position.z; */
            /* ee_traj_arrow_x.points[1].x = x_axis[0]; */
            /* ee_traj_arrow_x.points[1].y = x_axis[1]; */
            /* ee_traj_arrow_x.points[1].z = x_axis[2]; */

            /* // Update EE trajectory y-axis marker characteristics */
            ee_traj_arrow_y.header.stamp = this->get_clock()->now();
            ee_traj_arrow_y.id = j * 3;
            /* /1* ee_traj_arrow_y.pose.position.x = ee_htm.translation().x(); *1/ */
            /* /1* ee_traj_arrow_y.pose.position.y = ee_htm.translation().y(); *1/ */
            /* /1* ee_traj_arrow_y.pose.position.z = ee_htm.translation().z(); *1/ */
            /* /1* Eigen::Vector3d y_axis(0, 1, 0); *1/ */
            /* /1* axis_direction = (ee_orientation * y_axis).normalized(); *1/ */
            /* /1* ee_traj_arrow_y.pose.orientation.w = 1.0; *1/ */
            /* /1* ee_traj_arrow_y.pose.orientation.x = axis_direction.x(); *1/ */
            /* /1* ee_traj_arrow_y.pose.orientation.y = axis_direction.y(); *1/ */
            /* /1* ee_traj_arrow_y.pose.orientation.z = axis_direction.z(); *1/ */
            /* ee_traj_arrow_y.points.resize(2); */
            /* ee_traj_arrow_y.points[0].x = waypoint.position.x; */
            /* ee_traj_arrow_y.points[0].y = waypoint.position.y; */
            /* ee_traj_arrow_y.points[0].z = waypoint.position.z; */
            /* ee_traj_arrow_y.points[1].x = y_axis[0]; */
            /* ee_traj_arrow_y.points[1].y = y_axis[1]; */
            /* ee_traj_arrow_y.points[1].z = y_axis[2]; */

            /* // Update EE trajectory y-axis marker characteristics */
            ee_traj_arrow_z.header.stamp = this->get_clock()->now();
            ee_traj_arrow_z.id = j * 4;
            /* /1* ee_traj_arrow_z.pose.position.x = ee_htm.translation().x(); *1/ */
            /* /1* ee_traj_arrow_z.pose.position.y = ee_htm.translation().y(); *1/ */
            /* /1* ee_traj_arrow_z.pose.position.z = ee_htm.translation().z(); *1/ */
            /* /1* Eigen::Vector3d z_axis(0, 0, 1); *1/ */
            /* /1* axis_direction = (ee_orientation * z_axis).normalized(); *1/ */
            /* /1* ee_traj_arrow_z.pose.orientation.w = 1.0; *1/ */
            /* /1* ee_traj_arrow_z.pose.orientation.x = axis_direction.x(); *1/ */
            /* /1* ee_traj_arrow_z.pose.orientation.y = axis_direction.y(); *1/ */
            /* /1* ee_traj_arrow_z.pose.orientation.z = axis_direction.z(); *1/ */
            /* ee_traj_arrow_z.points.resize(2); */
            /* ee_traj_arrow_z.points[0].x = waypoint.position.x; */
            /* ee_traj_arrow_z.points[0].y = waypoint.position.y; */
            /* ee_traj_arrow_z.points[0].z = waypoint.position.z; */
            /* ee_traj_arrow_z.points[1].x = z_axis[0]; */
            /* ee_traj_arrow_z.points[1].y = z_axis[1]; */
            /* ee_traj_arrow_z.points[1].z = z_axis[2]; */

            // Store the marker
            ee_traj.markers.push_back(ee_traj_point);
            visual_tools_->publishAxis(ee_htm);
            /* ee_traj.markers.push_back(ee_traj_arrow_x); */
            /* ee_traj.markers.push_back(ee_traj_arrow_y); */
            /* ee_traj.markers.push_back(ee_traj_arrow_z); */
            /* std::cout << "DEBUG:AFTER PUSHING" << std::endl; */
            visual_tools_->trigger();
        }

        // Publish EE trajectory marker array
        rclcpp::Rate rate(4.0);
        for (size_t k = 0; k < 5; k++)
        {
            ee_traj_pub_->publish(ee_traj);
            rate.sleep();
        }

        serv_res->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<MoveItPlanAndVizServer>(node_options);
    node->start_and_spin_executor();

    while (rclcpp::ok())
    {
    }
    return 0;
}
