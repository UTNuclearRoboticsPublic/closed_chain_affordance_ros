#include "cca_ros/cca_ros_behavior.hpp"

namespace cca_ros_behavior
{
// Constructor implementation
CcaRosAction::CcaRosAction(const std::string &name, const BT::NodeConfig &config,
                           const rclcpp::NodeOptions &node_options, bool visualize_trajectory, bool execute_trajectory)
    : BT::StatefulActionNode(name, config),
      cca_ros::CcaRos(name, node_options, visualize_trajectory, execute_trajectory)

{
}

// Static method to provide the ports for BehaviorTree
BT::PortsList CcaRosAction::providedPorts() { return {BT::InputPort<std::string>("cca_task")}; }

// onStart() - Called once at the beginning of the action
BT::NodeStatus CcaRosAction::onStart()
{
    // Extract input
    // Task Description port is Required, if planner config and kinematic state are not provided, we'll use default.

    // Run CCAROS
    this->run_cc_affordance_planner(planner_config, task_description, motion_status_, start_config);
    start_time_ = std::chrono::steady_clock::now();
    // Initialize or perform actions at the start
    // Return BT::NodeStatus::RUNNING or other statuses as needed
    return BT::NodeStatus::RUNNING;
}

// onRunning() - Called repeatedly after onStart() until the action completes
BT::NodeStatus CcaRosAction::onRunning()
{
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count() > timeout_)
    {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for CCA motion to complete.");
        return BT::NodeStatus::FAILURE;
    }
    // Check CCA ROS action status

    rclcpp::spin_some(this->get_node_base_interface());

    if (motion_status_ != cca_ros::Status::SUCCEEDED)
    {

        RCLCPP_INFO(this->get_logger(), "CCARosAction successfully completed");
        return BT::NodeStatus::SUCCESS;
    }
    else if (*motion_status_ == cca_ros::Status::UNKNOWN)
    {
        RCLCPP_ERROR(this->get_logger(), "CCA Motion was interrupted mid-execution.");
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
    // Perform the running action logic
    // You can check for completion or cancellation here
    return BT::NodeStatus::RUNNING; // or other statuses like BT::NodeStatus::SUCCESS or BT::NodeStatus::FAILURE
}

// onHalted() - Called if the action was aborted by another node
void CcaRosAction::onHalted()
{
    // Perform any necessary cleanup or action when halted
    // E.g., canceling running tasks, stopping trajectories, etc.
}
} // namespace cca_ros_behavior
