#ifndef CCA_ROS_BEHAVIOR_HPP
#define CCA_ROS_BEHAVIOR_HPP

#include "cca_ros/cca_ros.hpp"
#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <string>

namespace cca_ros_behavior
{
class CcaRosAction : public BT::StatefulActionNode, public cca_ros::CcaRos
{
  public:
    CcaRosAction(const std::string &name, const BT::NodeConfig &config,
                 const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions(), bool visualize_trajectory = true,
                 bool execute_trajectory = false);

    // Static function to provide the ports for BehaviorTree
    static BT::PortsList providedPorts();

    // Invoked once at the beginning of the action
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, this method is constantly called until it returns something other than RUNNING
    BT::NodeStatus onRunning() override;

    // Callback to execute if the action was aborted by another node
    void onHalted() override;

  private:
    std::shared_ptr<cca_ros::Status> motion_status_ = std::make_shared<cca_ros::Status>(cca_ros::Status::UNKNOWN);
    std::chrono::time_point<std::chrono::steady_clock> start_time_;
    static constexpr int timeout_ = 60;
};

} // namespace cca_ros_behavior

#endif // CCA_ROS_BEHAVIOR_HPP
