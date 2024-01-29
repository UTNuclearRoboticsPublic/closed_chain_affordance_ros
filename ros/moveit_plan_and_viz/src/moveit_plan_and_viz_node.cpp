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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_plan_and_viz_server");

class MoveItPlanAndVizServer : public rclcpp::Node
{
  public:
    MoveItPlanAndVizServer(const rclcpp::NodeOptions &options)
        : Node("moveit_plan_and_viz_server", options), node_handle_(this->shared_from_this())
    {
        // Create and add node to executor
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(node_handle_); // Use shared_from_this() to keep node alive

        // Start executor thread
        executor_thread_ = std::thread([this]() { executor_->spin(); });

        // Create and advertise the service
        srv_ = this->create_service<moveit_plan_and_viz::srv::MoveItPlanAndViz>(
            "/moveit_plan_and_viz_server", &MoveItPlanAndVizServer::moveit_plan_and_viz_server_callback_);

        // Create a publisher to publish EE trajectory
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tool_trajectory_marker", 10);
    }

    ~MoveItPlanAndVizServer()
    {
        // Stop executor gracefully
        executor_->cancel();
        executor_thread_.join();
    }

  private:
    // variables
    rclcpp::Node::SharedPtr node_handle_; // Node handle member variable
    rclcpp::Service<moveit_plan_and_viz::srv::MoveItPlanAndViz>::SharedPtr srv_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;

    // methods
    void moveit_plan_and_viz_server_callback_(
        const std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Request> serv_req,
        std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Response> serv_res)
    {

        /* MoveIt Planning using planning_pipeline. The following excerpt of code
         * has been directly copied from MoveIt tutorial */
        /* -------------------------------------------------------------- */

        // MoveIt Planning
        // Before we can
        // load the planning_pipeline planner, we need two objects, a RobotModel and
        // a PlanningScene.
        //
        // We will start by instantiating a `RobotModelLoader`_ object, which will
        // look up the robot description on the ROS parameter server and construct a
        // :moveit_core:`RobotModel` for us to use.
        //
        // .. _RobotModelLoader:
        //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
        robot_model_loader::RobotModelLoaderPtr robot_model_loader(
            new robot_model_loader::RobotModelLoader(node_handle_, "robot_description"));

        // Using the RobotModelLoader, we can construct a planing scene monitor that
        // will create a planning scene, monitors planning scene diffs, and apply
        // the diffs to it's internal planning scene. We then call
        // startSceneMonitor, startWorldGeometryMonitor and startStateMonitor to
        // fully initialize the planning scene monitor
        planning_scene_monitor::PlanningSceneMonitorPtr psm(
            new planning_scene_monitor::PlanningSceneMonitor(node_handle_, robot_model_loader));

        /* listen for planning scene messages on topic /XXX and apply them to
                             the internal planning scene accordingly */
        psm->startSceneMonitor();
        /* listens to changes of world geometry, collision objects, and (optionally)
         * octomaps */
        psm->startWorldGeometryMonitor();
        /* listen to joint state updates as well as changes in attached collision
           objects and update the internal planning scene accordingly*/
        psm->startStateMonitor();

        /* We can also use the RobotModelLoader to get a robot model which contains
         * the robot's kinematic information */
        moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

        /* We can get the most up to date robot state from the PlanningSceneMonitor
           by locking the internal planning scene for reading. This lock ensures
           that the underlying scene isn't updated while we are reading it's state.
           RobotState's are useful for computing the forward and inverse kinematics
           of the robot among many other uses */
        moveit::core::RobotStatePtr robot_state(
            new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

        /* Create a JointModelGroup to keep track of the current robot pose and
           planning group. The Joint Model
           group is useful for dealing with one set of joints at a time such as a
           left arm or a end effector */
        const moveit::core::JointModelGroup *joint_model_group = robot_state->getJointModelGroup("arm");

        // We can now setup the PlanningPipeline object, which will use the ROS
        // parameter server to determine the set of request adapters and the
        // planning plugin to use
        planning_pipeline::PlanningPipelinePtr planning_pipeline(
            new planning_pipeline::PlanningPipeline(robot_model, node_handle_, "planning_plugin", "request_adapters"));

        // Visualization
        // ^^^^^^^^^^^^^
        // The package MoveItVisualTools provides many capabilities for visualizing
        // objects, robots, and trajectories in RViz as well as debugging tools such
        // as step-by-step introspection of a script.
        namespace rvt = rviz_visual_tools;
        moveit_visual_tools::MoveItVisualTools visual_tools(node_handle_, "arm0_base_link", "moveit_plan_and_viz", psm);
        visual_tools.deleteAllMarkers();

        /* Remote control is an introspection tool that allows users to step through
           a high level script via buttons and keyboard shortcuts in RViz */
        visual_tools.loadRemoteControl();

        /* RViz provides many types of markers, in this demo we will use text,
         * cylinders, and spheres*/
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.75;
        visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

        // We will now create a motion plan request for the arm
        planning_interface::MotionPlanRequest req;
        planning_interface::MotionPlanResponse res;
        req.group_name = "arm";

        // Visualization variables
        rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher =
            node_handle_->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
        moveit_msgs::msg::DisplayTrajectory display_trajectory;

        moveit_msgs::msg::MotionPlanResponse response;

        visual_tools.trigger();
        /* Wait for user input */
        visual_tools.prompt("Press next to plan the trajectory");

        // We ensure we plan from the current state of the robot
        /* static const std::string PLANNING_GROUP = "arm"; */
        /* moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP); */
        /* moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(); */
        // Retrieve the current planning scene
        const auto current_scene = psm->getPlanningScene();

        // Get the current robot state from the planning scene
        const auto current_state = current_scene->getCurrentState();

        std::vector<double> joint_group_positions_cur;
        current_state.copyJointGroupPositions(joint_model_group, joint_group_positions_cur);

        // Lock the planning scene and set the current state to the current robot
        // state just to be sure
        *robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState();

        //**** Planning loop
        const size_t lof_joint_traj = serv_req->joint_traj.points.size();
        /* robot_state::RobotStatePtr last_robot_state; */
        std::vector<geometry_msgs::msg::Pose> waypoints;

        for (size_t j = 0; j < lof_joint_traj; j++)
        {
            /* First, set the state in the planning scene to the final state of the
             * last plan */

            if (j == 0)
            {
                robot_state->setJointGroupPositions(joint_model_group, joint_group_positions_cur);
                moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);
            }
            else
            {

                robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(
                    response.trajectory_start);
                robot_state->setJointGroupPositions(joint_model_group,
                                                    response.trajectory.joint_trajectory.points.back().positions);
            }

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

            waypoints.push_back(waypoint);
            // Now, setup a joint space goal based on the info from service request
            std::vector<double> joint_group_positions;
            trajectory_msgs::msg::JointTrajectoryPoint &point = serv_req->joint_traj.points[j];

            for (size_t i = 0; i < point.positions.size(); i++)
            {
                joint_group_positions.push_back(point.positions[i]);
            }

            moveit::core::RobotState goal_state(*robot_state);

            goal_state.setJointGroupPositions(joint_model_group, joint_group_positions);
            moveit_msgs::msg::Constraints joint_goal =
                kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

            req.goal_constraints.clear();
            req.goal_constraints.push_back(joint_goal);

            // Before planning, we will need a Read Only lock on the planning scene so
            // that it does not modify the world representation while planning
            {
                planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
                /* Now, call the pipeline and check whether planning was successful. */
                planning_pipeline->generatePlan(lscene, req, res);
            }
            /* Check that the planning was successful */
            if (res.error_code_.val != res.error_code_.SUCCESS)
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not compute plan successfully");
                return;
            }
            /* Visualize the trajectory */
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing the trajectory");
            res.getMessage(response);
            /* visual_tools.deleteAllMarkers(); */
            display_trajectory.trajectory_start = response.trajectory_start;
            display_trajectory.trajectory.push_back(response.trajectory);
            // Now you should see two planned trajectories in series
            display_publisher->publish(display_trajectory);
            /* if (j != 0) { */
            /*   visual_tools.publishAxisLabeled( */
            /*       last_robot_state->getGlobalLinkTransform("arm0_fingers"), */
            /*       "start_pose"); */
            /*   visual_tools.publishAxisLabeled( */
            /*       robot_state->getGlobalLinkTransform("arm0_fingers"),
             * "target_pose"); */
            /* } */
            /* last_robot_state = robot_state; */
            visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
        }

        // Last point
        robot_state =
            planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
        robot_state->setJointGroupPositions(joint_model_group,
                                            response.trajectory.joint_trajectory.points.back().positions);
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

        waypoints.push_back(waypoint);
        /* -------------------------------------------------------------- */
        /* visual_tools.deleteAllMarkers(); */
        /* visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL); */
        /* for (std::size_t i = 0; i < waypoints.size(); ++i) */
        /*   visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i),
         */
        /*                                   rvt::SMALL); */
        rclcpp::Rate rate(4.0);
        // Populate markers with spheres at the waypoint locations
        int k = 0;
        while (k < 5)
        {
            visualization_msgs::msg::MarkerArray markers;
            for (size_t i = 0; i < waypoints.size(); ++i)
            {
                visualization_msgs::msg::Marker marker;
                marker.header.frame_id = "base_link"; // Replace with your robot's base frame
                marker.header.stamp = this->get_clock()->now();
                marker.ns = "trajectory";
                marker.id = i;
                marker.type = visualization_msgs::msg::Marker::SPHERE;
                marker.action = visualization_msgs::msg::Marker::ADD;
                marker.pose = waypoints[i]; // Assuming waypoints contain Pose information
                marker.scale.x = 0.01;
                marker.scale.y = 0.01;
                marker.scale.z = 0.01;
                marker.color.a = 1.0;
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;

                markers.markers.push_back(marker);
            }

            marker_pub_->publish(markers);
            rate.sleep();
            k++;
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
