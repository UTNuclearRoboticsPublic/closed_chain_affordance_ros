#ifndef CCA_ROS_ACTION_HPP
#define CCA_ROS_ACTION_HPP

#include "rclcpp_action/rclcpp_action.hpp"
#include "cca_ros_viz_msgs/action/cca_ros_action.hpp"  // Include your action definition
#include "cca_ros/cca_ros.hpp"  // Include your CcaRos node class
#include "cca_ros_action/visibility_control.h"

namespace cca_ros_action
{
    class CcaRosActionServer : public cca_ros::CcaRos
    {
    public:
        using CcaRosAction = cca_ros_viz_msgs::action::CcaRosAction;
        using GoalHandleCcaRosAction = rclcpp_action::ServerGoalHandle<CcaRosAction>;

        // Constructor that initializes the ROS 2 action server
	CCA_ROS_ACTION_CPP_PUBLIC
	explicit CcaRosActionServer(const std::string &node_name, const rclcpp::NodeOptions &node_options);

    private:
        rclcpp_action::Server<CcaRosAction>::SharedPtr action_server_;
	std::string as_server_name_;

        // Timeout duration
        int timeout_secs_;
        std::chrono::steady_clock::time_point start_time_;

        // Goal handling
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const CcaRosAction::Goal> goal);

        // Cancel handling
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleCcaRosAction> goal_handle);

        // Handle accepted goals and start execution in a separate thread
        void handle_accepted(const std::shared_ptr<GoalHandleCcaRosAction> goal_handle);

        // The actual execution of the goal, including timeout handling
        void execute_action(const std::shared_ptr<GoalHandleCcaRosAction> goal_handle);
    };
}  // namespace cca_ros_action

#endif  // CCA_ROS_ACTION_HPP
