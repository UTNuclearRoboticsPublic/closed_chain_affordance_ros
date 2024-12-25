#include "cca_ros/cca_ros_behavior.hpp"

namespace cca_ros_behavior
{
CcaRosAction::CcaRosAction(const std::string &name, const BT::NodeConfig &config,
                           const rclcpp::NodeOptions &node_options, bool visualize_trajectory, bool execute_trajectory)
    : BT::StatefulActionNode(name, config),
      cca_ros::CcaRos(name, node_options, visualize_trajectory, execute_trajectory)
{
    // Constructor initializes the base class and Behavior Tree node
}

CcaRosAction::~CcaRosAction()
{
    // Cleanup spinner thread
    if (spinner_thread_.joinable())
    {
        spinner_thread_.join();
    }
}

BT::PortsList CcaRosAction::providedPorts()
{
    // Define the ports required by this action node
    return {BT::InputPort<cca_ros::PlanningRequest>("cca_planning_request"),
            BT::InputPort<cca_ros::PlanningRequests>("cca_planning_requests")};
}

BT::NodeStatus CcaRosAction::onStart()
{
    // Spin this node in a separate thread to handle ROS communication
    spinner_thread_ = std::thread([this]() { rclcpp::spin(this->shared_from_this()); });

    BT::Expected<cca_ros::PlanningRequest> req = getInput<PlanningRequest>("cc_planning_request");
    BT::Expected<cca_ros::PlanningRequests> reqs = getInput<PlanningRequests>("cc_planning_requests");

    // Check if both inputs are provided, which is not allowed
    if (req.has_value() && reqs.has_value())
    {
        throw BT::runtime_error(
            "Error: Both [cca_planning_request] and [cca_planning_requests] cannot be provided simultaneously. "
            "Please specify only one.");
    }

    // Assign the final request based on which input is available
    auto cca_req =
        req.has_value()
            ? req.value() // Use the single request if provided
            : (reqs.has_value()
                   ? reqs.value() // Use multiple requests if provided
                   : throw BT::runtime_error("Error: One of [cca_planning_request] or [cca_planning_requests] must be "
                                             "provided, but neither was found."));

    // Point to the motion status from the request and use it to monitor this action
    motion_status_ = cca_req.motion_status;

    // Run the CCA planner using the extracted request data
    if (!this->run_cc_affordance_planner(cca_req.planner_config, cca_req.task_description, cca_req.motion_status,
                                         cca_req.start_config))
    {
        return BT::NodeStatus::FAILURE; // Return failure if planning fails
    }

    // Record start time to monitor for timeout
    start_time_ = std::chrono::steady_clock::now();

    return BT::NodeStatus::RUNNING; // Return running status to indicate ongoing action
}

BT::NodeStatus CcaRosAction::onRunning()
{
    // Check if the CCA action has timed out
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count() > timeout_)
    {
        RCLCPP_ERROR(this->get_logger(), "Timed out waiting for CCA action request to complete.");
        return BT::NodeStatus::FAILURE;
    }

    // Check the status of the CCA action
    if (motion_status_ == cca_ros::Status::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "CCA action successfully completed");
        return BT::NodeStatus::SUCCESS;
    }
    else if (*motion_status_ == cca_ros::Status::UNKNOWN)
    {
        RCLCPP_ERROR(this->get_logger(), "CCA action was interrupted mid-execution.");
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        return BT::NodeStatus::RUNNING; // Action is still running
    }
}

void CcaRosAction::onHalted()
{
    // Attempt to cancel trajectory execution if the action is halted
    this->cancel_execution();
}
} // namespace cca_ros_behavior
