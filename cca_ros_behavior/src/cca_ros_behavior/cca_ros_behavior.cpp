#include "cca_ros_behavior/cca_ros_behavior.hpp"

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
    return {BT::InputPort<std::shared_ptr<cca_ros::PlanningRequest>>("cca_planning_request"),
            BT::InputPort<std::shared_ptr<cca_ros::PlanningRequests>>("cca_planning_requests")};
}

BT::NodeStatus CcaRosAction::onStart()
{
    // Define type aliases for readability
    using PlanningRequestPtr = std::shared_ptr<cca_ros::PlanningRequest>;
    using PlanningRequestsPtr = std::shared_ptr<cca_ros::PlanningRequests>;

    // Spin this node in a separate thread to handle ROS communication
    spinner_thread_ = std::thread([this]() { rclcpp::spin(this->shared_from_this()); });

    // Attempt to get inputs from the ports
    BT::Expected<PlanningRequestPtr> req = getInput<PlanningRequestPtr>("cc_planning_request");
    BT::Expected<PlanningRequestsPtr> reqs = getInput<PlanningRequestsPtr>("cc_planning_requests");

    // Check for !XOR between the two ports
    if (req.has_value() == reqs.has_value())
    {
        throw BT::RuntimeError(
            "Error: One of [cca_planning_request] or [cca_planning_requests] must be provided but not both. "
            "Please specify one and only one.");
    }

    // Assign the final request based on available input. One of them is available at this point.
    using RequestVariant = std::variant<PlanningRequestPtr, PlanningRequestsPtr>;
    auto cca_req_variant = req.has_value() ? RequestVariant(req.value())   // Use the single request if provided
                                           : RequestVariant(reqs.value()); // Otherwise, use multiple requests

    // Lambda to process the requests
    auto process_request = [this](const auto &cca_req) -> BT::NodeStatus {
        // Point to the motion status from the request and use it to monitor this action
        status_ = cca_req->status;

        // Run the CCA planner using the extracted request data
        if (!this->plan_visualize_and_execute(*cca_req))
        {
            return BT::NodeStatus::FAILURE; // Return failure if planning fails
        }

        return BT::NodeStatus::RUNNING; // Return running status if planning succeeds
    };

    // Use std::visit to handle both types
    return std::visit([this, &process_request](auto &cca_req) { return process_request(cca_req); }, cca_req_variant);
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
    if (*status_ == cca_ros::Status::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "CCA action successfully completed");
        return BT::NodeStatus::SUCCESS;
    }
    else if (*status_ == cca_ros::Status::UNKNOWN)
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
