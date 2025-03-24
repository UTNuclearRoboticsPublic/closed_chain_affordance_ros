#ifndef CCA_ROS_ACTION_HPP
#define CCA_ROS_ACTION_HPP

#include "cca_ros_action/visibility_control.h"
#include <cca_ros/cca_ros.hpp>
#include <cca_ros_msgs/action/cca_ros_action.hpp>
#include <cca_ros_util/cca_ros_util.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

namespace cca_ros_action
{
using CcaRosAction = cca_ros_msgs::action::CcaRosAction;

constexpr const char *CCA_ROS_AS_NAME = "/cca_ros_action"; ///< Name of the action server

/**
 * @class CcaRosActionServer
 * @brief A ROS 2 action server for handling CCA ROS actions.
 */
class CcaRosActionServer : public cca_ros::CcaRos
{
  public:
    using GoalHandleCcaRosActionServer = rclcpp_action::ServerGoalHandle<CcaRosAction>;

    /**
     * @brief Constructs a CcaRosActionServer object.
     *
     * @param node_name Name of the ROS node.
     * @param node_options Options for the ROS node.
     */
    CCA_ROS_ACTION_CPP_PUBLIC
    explicit CcaRosActionServer(const std::string &node_name, const rclcpp::NodeOptions &node_options);

  private:
    rclcpp_action::Server<CcaRosAction>::SharedPtr action_server_; ///< Action server to execute CCA ROS action

    static constexpr int TIMEOUT_SECS_ = 60; ///< Timeout in seconds before aborting the goal due to inactivity

    /**
     * @brief Handles incoming goal requests.
     *
     * @param uuid Unique identifier for the goal.
     * @param goal Shared pointer to the goal message.
     * @return GoalResponse indicating acceptance or rejection of the goal.
     */
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                            std::shared_ptr<const CcaRosAction::Goal> goal);

    /**
     * @brief Handles goal cancelation requests.
     *
     * @param goal_handle Shared pointer to the goal handle representing the goal to be canceled.
     * @return CancelResponse indicating acceptance or rejection of the cancel request.
     */
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle);

    /**
     * @brief Handles accepted goals and starts execution in a separate thread.
     *
     * @param goal_handle Shared pointer to the goal handle representing the accepted goal.
     */
    void handle_accepted(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle);

    /**
     * @brief Executes the goal, performing the action with timeout handling.
     *
     * @param goal_handle Shared pointer to the goal handle representing the executing goal.
     */
    void execute_action(const std::shared_ptr<GoalHandleCcaRosActionServer> goal_handle);
};

/**
 * @class CcaRosActionClient
 * @brief A ROS 2 action client for sending goals to the CCA ROS action server.
 */
class CcaRosActionClient : public rclcpp::Node
{
  public:
    using GoalHandleCcaRosActionClient = rclcpp_action::ClientGoalHandle<CcaRosAction>;

    /**
     * @brief Constructs a CcaRosActionClient object.
     */
    explicit CcaRosActionClient();

    /**
     * @brief Sends a planning request to the CCA ROS action server for execution.
     *
     * @param req The planning request containing parameters for the action.
     */
    void send_goal(const cca_ros::PlanningRequest &req);

    /**
     * @brief Cancels the current goal being executed.
     */
    void cancel_goal();

  private:
    rclcpp_action::Client<CcaRosAction>::SharedPtr
        action_client_; ///< Client for sending goals to the CCA ROS action server
    std::shared_future<GoalHandleCcaRosActionClient::SharedPtr> goal_future_; ///< Future representing the goal handle

    /**
     * @brief Callback for goal response from the server.
     *
     * @param goal_handle Shared pointer to the goal handle.
     */
    void goal_response_cb_(const GoalHandleCcaRosActionClient::SharedPtr &goal_handle);

    /**
     * @brief Callback for receiving the result of goal execution.
     *
     * @param result WrappedResult containing the result of the action.
     */
    void result_cb_(const GoalHandleCcaRosActionClient::WrappedResult &result);
};

} // namespace cca_ros_action

#endif // CCA_ROS_ACTION_HPP
