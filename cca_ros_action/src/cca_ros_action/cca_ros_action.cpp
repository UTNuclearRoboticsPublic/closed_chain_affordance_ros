#include "cca_ros_action/cca_ros_action.hpp"
#include <cca_ros/cca_ros.hpp>
namespace cca_ros_action
{
CcaRosActionServer::CcaRosActionServer(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : cca_ros::CcaRos(node_name, node_options), node_logger_(this->get_node()->get_logger())
{
    // Initialize the action server
    action_server_ = rclcpp_action::create_server<CcaRosAction>(
        this->get_node(), CCA_ROS_AS_NAME,
        std::bind(&CcaRosActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&CcaRosActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&CcaRosActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_logger_, "Action server '%s' initialized.", CCA_ROS_AS_NAME);
}

rclcpp_action::GoalResponse CcaRosActionServer::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                            std::shared_ptr<const CcaRosAction::Goal> goal)
{
    RCLCPP_INFO(node_logger_, "Received a new goal request on action server '%s' with affordance goal: %.4f.",
                CCA_ROS_AS_NAME, goal->req.task_description.goal.affordance);
    (void)uuid;

    // Validate the goal (you can add additional checks)
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CcaRosActionServer::handle_cancel(
    const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle)
{
    RCLCPP_INFO(node_logger_, "Received a cancel request on action server '%s'.", CCA_ROS_AS_NAME);
    (void)goal_handle;

    this->cancel_execution();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CcaRosActionServer::handle_accepted(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle)
{
    RCLCPP_INFO(node_logger_, "Accepted goal on action server '%s'.", CCA_ROS_AS_NAME);

    // Safely launch the execution in a background thread
    std::thread([this, goal_handle]() {
        this->execute_action(goal_handle);
    }).detach();
}

void CcaRosActionServer::execute_action(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(node_logger_, "Executing action on server '%s'.", CCA_ROS_AS_NAME);

    // Convert the ROS message to cca_ros planning request struct
    cca_ros::PlanningRequest req = cca_ros_util::convert_cca_ros_action_to_req(goal->req);

    // Print the log
    const std::stringstream req_log = cca_ros_util::log_cca_planning_request(req);
    RCLCPP_INFO(node_logger_, "%s", req_log.str().c_str());
    
    const cca_ros::PlanningResponse response = this->plan(req);
    if (!response.result.success)
    {
        RCLCPP_ERROR(node_logger_, "Execution failed on action server '%s'.", CCA_ROS_AS_NAME);
        goal_handle->abort(std::make_shared<CcaRosAction::Result>());
        return;
    }

    auto start_time = std::chrono::steady_clock::now();

    while (rclcpp::ok())
    {
        auto current_time = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > TIMEOUT_SECS_)
        {
            RCLCPP_ERROR(node_logger_, "Timed out waiting for action request to complete on server '%s'.",
                         CCA_ROS_AS_NAME);
            goal_handle->abort(std::make_shared<CcaRosAction::Result>());
            return;
        }

        if (*response.status == cca_ros::Status::SUCCEEDED)
        {
            RCLCPP_INFO(node_logger_, "Action successfully completed on server '%s'.", CCA_ROS_AS_NAME);
            goal_handle->succeed(std::make_shared<CcaRosAction::Result>());
            return;
        }
        else if (*response.status == cca_ros::Status::UNKNOWN)
        {
            RCLCPP_ERROR(node_logger_, "Action was interrupted mid-execution on server '%s'.", CCA_ROS_AS_NAME);
            goal_handle->abort(std::make_shared<CcaRosAction::Result>());
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

CcaRosActionClient::CcaRosActionClient() : rclcpp::Node("cca_ros_action_client"), node_logger_(this->get_logger())
{

    action_client_ = rclcpp_action::create_client<CcaRosAction>(this, CCA_ROS_AS_NAME);

}
void CcaRosActionClient::send_goal(const cca_ros::PlanningRequest &req)
{
    // Create goal
    auto goal_msg = cca_ros_msgs::action::CcaRosAction::Goal();
    goal_msg.req = cca_ros_util::convert_req_to_cca_ros_action(req);

    if (!this->action_client_->wait_for_action_server())
    {
        RCLCPP_ERROR(node_logger_, "%s action server not available after waiting", CCA_ROS_AS_NAME);
        rclcpp::shutdown();
    }

    RCLCPP_INFO(node_logger_, "Sending goal to %s action server.", CCA_ROS_AS_NAME);

    using namespace std::placeholders;
    auto send_goal_options = rclcpp_action::Client<CcaRosAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&CcaRosActionClient::goal_response_cb_, this, _1);
    send_goal_options.result_callback = std::bind(&CcaRosActionClient::result_cb_, this, _1);
    goal_future_ = this->action_client_->async_send_goal(goal_msg, send_goal_options);
}

void CcaRosActionClient::cancel_goal()
{
    if (goal_future_.valid())
    {
        auto goal_handle = goal_future_.get(); // Extracts the goal handle
        if (goal_handle)
        {
            action_client_->async_cancel_goal(goal_handle); // Cancel the goal
        }
        else
        {
            RCLCPP_WARN(node_logger_, "Cannot cancel goal because it was not accepted by the server: %s",
                        CCA_ROS_AS_NAME);
        }
    }
    else
    {
        RCLCPP_WARN(node_logger_, "Unable to cancel %s server goal due to the goal future being invalid ",
                    CCA_ROS_AS_NAME);
    }
}
void CcaRosActionClient::goal_response_cb_(const GoalHandleCcaRosActionClient::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_logger_, "Goal was rejected by action server, %s", CCA_ROS_AS_NAME);
    }
    else
    {
        RCLCPP_INFO(node_logger_, "Goal accepted by %s server, waiting for result", CCA_ROS_AS_NAME);
    }
}

void CcaRosActionClient::result_cb_(const GoalHandleCcaRosActionClient::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_logger_, "Goal was aborted by %s server", CCA_ROS_AS_NAME);
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_logger_, "Goal was canceled by %s server", CCA_ROS_AS_NAME);
        return;
    default:
        RCLCPP_ERROR(node_logger_, "Unknown result code from %s server", CCA_ROS_AS_NAME);
        return;
    }
}

} // namespace cca_ros_action
