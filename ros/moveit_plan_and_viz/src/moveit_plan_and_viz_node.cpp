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
    MoveItPlanAndVizServer(const rclcpp::NodeOptions &options) : Node("moveit_plan_and_viz_server", options)
    {
        // Create and add node to executor
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor_->add_node(this->shared_from_this()); // Use shared_from_this() to keep node alive

        // Start executor thread
        executor_thread_ = std::thread([this]() { executor_->spin(); });

        // Create and advertise the service
        srv_ = create_service<moveit_plan_and_viz::srv::MoveItPlanAndViz>(
            "/moveit_plan_and_viz_server", &MoveItPlanAndVizServer::moveit_plan_and_viz_server_callback_, this);

        // Create a publisher to publish EE trajectory
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            service_namespace + "/tool_trajectory_marker", 10);
    }

    ~MoveItPlanAndVizServer()
    {
        // Stop executor gracefully
        executor_->cancel();
        executor_thread_.join();
    }

  private:
    // variables
    rclcpp::Service<moveit_plan_and_viz::srv::MoveItPlanAndViz>::SharedPtr srv_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread executor_thread_;

    // methods
    void moveit_plan_and_viz_server_callback_(
        const std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Request> request,
        std::shared_ptr<moveit_plan_and_viz::srv::MoveItPlanAndViz::Response> response)
    {
        // Implement service request processing and response population
        // using MoveIt and other relevant libraries.
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
