#ifndef CCA_ROS_ACTION_HPP
#define CCA_ROS_ACTION_HPP

#include "cca_ros/cca_ros.hpp" // Include your CcaRos node class
#include "cca_ros_action/visibility_control.h"
#include "cca_ros_viz_msgs/action/cca_ros_action.hpp" // Include your action definition
#include "rclcpp_action/rclcpp_action.hpp"
#include <cca_ros_util/cca_ros_util.hpp>
#include <cca_ros_viz_msgs/action/cca_ros_action.hpp>
#include <memory>
#include <thread>

namespace cca_ros_action
{
using CcaRosAction = cca_ros_viz_msgs::action::CcaRosAction;

constexpr const char *CCA_ROS_AS_NAME = "/cca_ros_action"; ///<-- Name of the action server

class CcaRosActionServer : public cca_ros::CcaRos
{
  public:
    using GoalHandleCcaRosActionServer = rclcpp_action::ServerGoalHandle<CcaRosAction>;

    // Constructor that initializes the ROS 2 action server
    CCA_ROS_ACTION_CPP_PUBLIC
    explicit CcaRosActionServer(const std::string &node_name, const rclcpp::NodeOptions &node_options);

  private:
    rclcpp_action::Server<CcaRosAction>::SharedPtr action_server_;

    // Timeout duration
    int timeout_secs_;
    std::chrono::steady_clock::time_point start_time_;

    // Goal handling
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const CcaRosAction::Goal> goal);

    // Cancel handling
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle);

    // Handle accepted goals and start execution in a separate thread
    void handle_accepted(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle);

    // The actual execution of the goal, including timeout handling
    void execute_action(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle);
};

class CcaRosActionClient : public rclcpp::Node
{
  public:
    using GoalHandleCcaRosActionClient = rclcpp_action::ClientGoalHandle<CcaRosAction>;
    explicit CcaRosActionClient();
    void send_goal(const cca_ros::PlanningRequest &req);
    void cancel_goal();

  private:
    rclcpp_action::Client<cca_ros_viz_msgs::action::CcaRosAction>::SharedPtr
        action_client_; ///< Client for planning and executing with CCA
    std::shared_future<GoalHandleCcaRosActionClient::SharedPtr> goal_future_; ///< Goal handle future for the CCA action

    void goal_response_cb_(const GoalHandleCcaRosActionClient::SharedPtr &goal_handle);
    void result_cb_(const GoalHandleCcaRosActionClient::WrappedResult &result);
};
} // namespace cca_ros_action

#endif // CCA_ROS_ACTION_HPP
