#ifndef CCA_ROS_BEHAVIOR_HPP
#define CCA_ROS_BEHAVIOR_HPP

#include "cca_ros/cca_ros.hpp"
#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <string>
#include <thread>

namespace cca_ros_behavior
{
/**
 * @brief A Behavior Tree action node for planning , visualizing, and executing robot joint trajectories using the
 * Closed-chain Affordance (CCA) framework.
 *
 * The action node requires the planning request(s) to be sent via the input port "cca_planning_request" or
 * "cca_planning_requests."
 */
class CcaRosAction : public BT::StatefulActionNode, public cca_ros::CcaRos
{
  public:
    /**
     * @brief Constructor for the CcaRosAction class.
     *
     * @param name The name of the action node.
     * @param config Configuration for the Behavior Tree node.
     * @param node_options Node options for the ROS 2 node (default is empty).
     * @param visualize_trajectory Whether to visualize the planned trajectory (default is true).
     * @param execute_trajectory Whether to execute the planned trajectory on the robot (default is false).
     */
    CcaRosAction(const std::string &name, const BT::NodeConfig &config,
                 const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions(), bool visualize_trajectory = true,
                 bool execute_trajectory = false);

    /**
     * @brief Destructor for the CcaRosAction class.
     */
    ~CcaRosAction();

    /**
     * @brief Returns the list of ports required by this action node.
     *
     * This action node requires one of the following input ports:
     * - "cca_planning_request" or "cca_planning_requests", both of which contain task description(s), optional planner
     * configuration(s), start state, and a shared pointer for monitoring status.
     *
     * @return A list of input ports used by the action node.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Initializes the planning process by extracting relevant information from the input port and calling the
     * planner.
     *
     * @return The status of the node after starting.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Monitors the status of CCA planning, visualization, and execution while the action is running.
     *
     * @return The current status of the node (RUNNING, SUCCESS, or FAILURE).
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Attempts to cancel the trajectory execution if the action is aborted.
     */
    void onHalted() override;

  private:
    std::thread spinner_thread_;                                    /**< Thread to spin the node. */
    std::shared_ptr<cca_ros::Status> motion_status_{nullptr};       /**< To check the status of the CCA action. */
    std::chrono::time_point<std::chrono::steady_clock> start_time_; /**< To monitor the timeout. */
    static constexpr int timeout_ = 60; /**< Timeout duration for the CCA action (in seconds). */
};
} // namespace cca_ros_behavior

#endif // CCA_ROS_BEHAVIOR_HPP
