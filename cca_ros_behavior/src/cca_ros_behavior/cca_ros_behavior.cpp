#include "cca_ros_behavior/cca_ros_behavior.hpp"
#include <cca_ros/cca_ros.hpp>

namespace cca_ros_behavior
{
CcaRosAction::CcaRosAction(const std::string &name, const BT::NodeConfig &config,
                           const rclcpp::NodeOptions &node_options)
    : BT::StatefulActionNode(name, config)
{
    // Spin this node in a separate thread to handle ROS communication
    node_ = std::make_shared<cca_ros::CcaRos>(name, node_options);
    spinner_thread_ = std::jthread([this]() { rclcpp::spin(node_); });
}

BT::PortsList CcaRosAction::providedPorts()
{
    // Define the ports required by this action node
    return {BT::InputPort<std::shared_ptr<cca_ros::PlanningRequest>>("cca_planning_request"),
            BT::InputPort<std::shared_ptr<std::vector<cca_ros::PlanningRequest>>>("cca_planning_requests"),
            BT::OutputPort<std::shared_ptr<cca_ros::PlanningResponse>>("cca_planning_response")};
}

BT::NodeStatus CcaRosAction::onStart()
{
    // Define type aliases for readability
    using PlanningRequestPtr = std::shared_ptr<cca_ros::PlanningRequest>;
    using PlanningRequestsPtr = std::shared_ptr<std::vector<cca_ros::PlanningRequest>>;

    // Attempt to get inputs from the ports
    BT::Expected<PlanningRequestPtr> req = getInput<PlanningRequestPtr>("cca_planning_request");
    BT::Expected<PlanningRequestsPtr> reqs = getInput<PlanningRequestsPtr>("cca_planning_requests");

    // Check for !XOR between the two ports
    if (req.has_value() == reqs.has_value())
    {
        throw BT::RuntimeError(
            "Error: Either both or none of the [cca_planning_request] or [cca_planning_requests] ports have value. "
            "Please specify one and only one.");
    }

    // Process based on which input is available
    cca_ros::PlanningResponse response;

    if (req.has_value())
    {
        timeout_ = req.value()->execution_timeout;
        response = node_->plan(*req.value());
    }
    else // reqs.has_value()
    {
        timeout_ = reqs.value()->front().execution_timeout;
        response = node_->plan(*reqs.value());
    }

    // Record start time to monitor timeout
    start_time_ = std::chrono::steady_clock::now();

    // Put response in output port regardless of success or failure
    const auto response_ptr = std::make_shared<cca_ros::PlanningResponse>(response);
    setOutput("cca_planning_response", response_ptr);

    // Store a pointer to the status for monitoring in onRunning
    status_ = response.status;

    // Return failure if planning fails
    if (!response.result.success)
    {
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::RUNNING; // Return running status if planning succeeds
}

BT::NodeStatus CcaRosAction::onRunning()
{
    // Check if the CCA action has timed out
    auto current_time = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_) > timeout_)
    {
        RCLCPP_ERROR(node_->get_logger(), "Timed out waiting for CCA action request to complete.");
        return BT::NodeStatus::FAILURE;
    }

    // Check the status of the CCA action
    if (*status_ == cca_ros::Status::SUCCEEDED)
    {
        RCLCPP_INFO(node_->get_logger(), "CCA action successfully completed");
        return BT::NodeStatus::SUCCESS;
    }
    else if (*status_ == cca_ros::Status::FAILED)
    {
        RCLCPP_ERROR(node_->get_logger(), "CCA action may have been canceled or aborted.");
        return BT::NodeStatus::FAILURE;
    }
    else if (*status_ == cca_ros::Status::UNKNOWN)
    {
        RCLCPP_ERROR(node_->get_logger(), "CCA action was interrupted mid-execution.");
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
    node_->cancel_execution();
}
} // namespace cca_ros_behavior
