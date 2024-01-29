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

class MoveItPlanAndVizServer : public rclcpp::Node
{
  public:
    MoveItPlanAndVizServer(const rclcpp::NodeOptions &options)
        : Node("moveit_plan_and_viz_server_node", options),
          node_handle_(this->shared_from_this()),
          node_logger_(rclcpp::get_logger(node_handle_->get_name())),
          executor_(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
    {
        // Add node to executor and start executor thread
        executor_->add_node(node_handle_);
        executor_thread_ = std::thread([this]() { executor_->spin(); });

        // Create and advertise planning and visualization service
        srv_ = this->create_service<moveit_plan_and_viz::srv::MoveItPlanAndViz>(
            "/moveit_plan_and_viz_server", std::bind(&MoveItPlanAndVizServer::moveit_plan_and_viz_server_callback_,
                                                     this, std::placeholders::_1, std::placeholders::_2));

        // Create a publisher to publish EE trajectory
        ee_traj_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/ee_trajectory", 10);

        // Initialize the publisher to show moveit planned path
        moveit_planned_path_pub_ =
            this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
    }

    ~MoveItPlanAndVizServer()
    {
        // Stop executor gracefully
        executor_->cancel();
        executor_thread_.join();
    }

  private:
    // Variables
    rclcpp::Node::SharedPtr node_handle_; // handle for the node
    rclcpp::Logger node_logger_;          // logger associated with the node
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
        executor_;                // executor needed for MoveIt robot state checking
    std::thread executor_thread_; // thread
    rclcpp::Service<moveit_plan_and_viz::srv::MoveItPlanAndViz>::SharedPtr
        srv_; // joint traj plan and visualization service
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ee_traj_pub_; // publisher for EE trajectory
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
        moveit_planned_path_pub_; // publisher to show moveit planned path

    // Methods
    void moveit_plan_and_viz_server_callback_(
        const std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Request> serv_req,
        std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Response> serv_res)
    {

        /* MoveIt Planning using planning_pipeline. Most of the code here has been taken from the MoveIt tutorial*/

        // Basic housekeeping for planning, need robot model and planning scene monitor, which observes changes in
        // planning scene
        robot_model_loader::RobotModelLoaderPtr robot_model_loader(
            new robot_model_loader::RobotModelLoader(node_handle_, "robot_description"));

        planning_scene_monitor::PlanningSceneMonitorPtr psm(
            new planning_scene_monitor::PlanningSceneMonitor(node_handle_, robot_model_loader));

        // Start listening to changes in the planning scene
        psm->startSceneMonitor();
        psm->startWorldGeometryMonitor(); // listens to world geometry, collision objects and (optionally) octomap
                                          // changes
        psm->startStateMonitor();         // listens to joint state updates and attached collision object changes

        // Get robot model
        moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

        // Create planning pipeline object using the robot model
        planning_pipeline::PlanningPipelinePtr planning_pipeline(
            new planning_pipeline::PlanningPipeline(robot_model, node_handle_, "planning_plugin", "request_adapters"));

        // Visualization using MoveIt Visual Tools
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(node_handle_, "arm0_base_link", "moveit_plan_and_viz", psm);
        visual_tools.deleteAllMarkers();

        visual_tools.loadRemoteControl(); // load introspection tool so we could step through high-level scripts

        // Show text in Rviz
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools.publishText(text_pose, "MoveIt Plan and Viz", rvt::WHITE, rvt::XLARGE);

        // Create motion plan request
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;
        req.group_name = "arm";

        // Moveit messages to visualize planned path and hold planning pipeline response
        moveit_msgs::msg::DisplayTrajectory display_trajectory;
        moveit_msgs::msg::MotionPlanResponse response;

        // Prompt user to press next to start planning trajectory
        visual_tools.trigger();
        visual_tools.prompt("Press next to plan the trajectory");

        // To ensure we plan from the current state of the robot, get the most up-to-date state
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(
            planning_scene_monitor::LockedPlanningSceneRO(psm)
                ->getCurrentState())); // planning scene is locked while reading robot
                                       // state to ensure no changes happen during the reading
        moveit::core::robotStateToRobotStateMsg(
            *robot_state, req.start_state); // update the planning request start state to be this current state

        // Create a JointModelGroup to keep track of the group we are planning for
        const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup("arm");

        // Extract the joint trajectory from the service request and plan and visualize for every subsequent points in
        // the trajectory
        const size_t lof_joint_traj = serv_req->joint_traj.points.size(); // length of the trajectory

        // EE trajectory markers
        visualization_msgs::msg::MarkerArray ee_traj;  // entire trajectory
        visualization_msgs::msg::Marker ee_traj_point; // a point within trajectory
        ee_traj_point.header.frame_id = "base_link";   // frame of reference for markers
        ee_traj_point.ns = "trajectory";
        ee_traj_point.type = visualization_msgs::msg::Marker::SPHERE;
        ee_traj_point.action = visualization_msgs::msg::Marker::ADD;
        ee_traj_point.scale.x = 0.01;
        ee_traj_point.scale.y = 0.01;
        ee_traj_point.scale.z = 0.01;
        ee_traj_point.color.a = 1.0;
        ee_traj_point.color.r = 0.0;
        ee_traj_point.color.g = 1.0;
        ee_traj_point.color.b = 0.0;

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
            Eigen::Isometry3d eigenTransform = robot_state->getGlobalLinkTransform("arm0_tool0");
            geometry_msgs::msg::Pose waypoint;
            // Set the translation (position) values
            waypoint.position.x = eigenTransform.translation().x();
            waypoint.position.y = eigenTransform.translation().y();
            waypoint.position.z = eigenTransform.translation().z();
            // Set the rotation (orientation) values (quaternion)
            Eigen::Quaterniond eigenQuaternion(eigenTransform.rotation());
            waypoint.orientation.x = eigenQuaternion.x();
            waypoint.orientation.y = eigenQuaternion.y();
            waypoint.orientation.z = eigenQuaternion.z();
            waypoint.orientation.w = eigenQuaternion.w();

            // Update EE trajectory marker characteristics
            ee_traj_point.header.stamp = this->get_clock()->now();
            ee_traj_point.id = j;
            ee_traj_point.pose = waypoint; // Assuming waypoints contain Pose information

            // Store the marker
            ee_traj.markers.push_back(ee_traj_point);
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

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
