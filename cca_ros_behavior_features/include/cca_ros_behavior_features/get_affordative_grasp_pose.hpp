///////////////////////////////////////////////////////////////////////////////
//      Title     : get_affordative_grasp_pose.hpp
//      Project   : cca_ros_behavior_features
//      Created   : 2026
//      Author    : Crasun Jans
///////////////////////////////////////////////////////////////////////////////

#ifndef GET_AFFORDATIVE_GRASP_POSE_HPP_
#define GET_AFFORDATIVE_GRASP_POSE_HPP_

#include <behaviortree_cpp/action_node.h>
#include <cca_ros/cca_ros.hpp>
#include <cca_ros_features/cca_ros_features.hpp>
#include <chrono>
#include <future>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>

namespace cca_ros_behavior_features
{

/**
 * @brief Behavior Tree action node that finds the first affordative grasp pose
 *        from a set of candidate grasp poses by planning in parallel.
 *
 * For each candidate grasp pose, a dedicated CcaRos planner is spawned in its
 * own thread and attempts to plan a full grasp sequence:
 *   1. A whole-body controller (WBC) approach
 *   2. An arm approach (seeded from the WBC end state)
 *   3. An arm grab
 *
 * The node returns the first affordative grasp pose — i.e. the first candidate
 * for which all three plans succeed — or FAILURE if no candidate yields a valid
 * plan within the timeout.
 *
 * @note Both WBC and arm planning requests are assumed to contain gripper goals.
 * @note The rclcpp::Node::SharedPtr is read from the blackboard key @c "node".
 *
 * @par BT Ports
 * | Direction | Name                   | Type                                             | Description                                      |
 * |-----------|------------------------|--------------------------------------------------|--------------------------------------------------|
 * | Input     | wbc_approach_req       | std::shared_ptr<cca_ros::PlanningRequest>        | Planning request for the WBC approach motion     |
 * | Input     | arm_approach_req       | std::shared_ptr<cca_ros::PlanningRequest>        | Planning request for the arm approach motion     |
 * | Input     | arm_grab_req           | std::shared_ptr<cca_ros::PlanningRequest>        | Planning request for the arm grab motion         |
 * | Input     | grasp_poses            | std::shared_ptr<geometry_msgs::msg::PoseArray>   | Array of candidate grasp poses to evaluate       |
 * | Output    | affordative_grasp_pose | std::shared_ptr<geometry_msgs::msg::PoseStamped> | First grasp pose for which all plans succeeded   |
 */
class GetAffordativeGraspPose : public BT::StatefulActionNode
{
  public:
    /**
     * @brief Construct a GetAffordativeGraspPose node.
     * @param name   Name of the BT node.
     * @param config BT node configuration containing the blackboard and port mappings.
     */
    GetAffordativeGraspPose(const std::string &name, const BT::NodeConfig &config);

    /**
     * @brief Returns the list of input and output ports for this BT node.
     * @return BT::PortsList containing all declared ports.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Reads input ports, validates them, and launches the parallel planning search asynchronously.
     *
     * @throws BT::RuntimeError if any required input port is missing or invalid.
     * @return BT::NodeStatus::RUNNING after launching the async planning task.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Polls the async planning task for completion on each BT tick.
     *
     * @return BT::NodeStatus::RUNNING while planning is in progress.
     * @return BT::NodeStatus::SUCCESS if an affordative grasp pose was found; sets the output port.
     * @return BT::NodeStatus::FAILURE if no valid plan was found within the timeout.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Waits for the async planning task to finish before releasing resources.
     */
    void onHalted() override;

  private:
    rclcpp::Node::SharedPtr node_; ///< ROS 2 node obtained from the blackboard key "node"
    std::shared_ptr<cca_ros::CcaRosContext> cca_ros_context_; ///< Context for CCA ROS
    std::future<std::optional<geometry_msgs::msg::PoseStamped>> result_future_; ///< Async planning result
    static constexpr std::chrono::milliseconds timeout_{100}; ///< Maximum time to wait for any planning thread
    std::stop_source stop_source_; ///< Stop source to signal get_affordative_grasp_pose to halt
};

} // namespace cca_ros_behavior_features

#endif // GET_AFFORDATIVE_GRASP_POSE_HPP_
