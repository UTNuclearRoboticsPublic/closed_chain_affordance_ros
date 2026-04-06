#ifndef GET_AFFORDATIVE_GRASP_POSE_HPP
#define GET_AFFORDATIVE_GRASP_POSE_HPP

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <atomic>
#include <behaviortree_cpp/action_node.h>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <condition_variable>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <tf2_eigen/tf2_eigen.hpp>
#include <thread>
#include <vector>
#include <cca_ros_features/cca_ros_features.hpp>

/**
 * @brief Behavior Tree action node that finds the first affordative grasp pose
 *        from a set of candidate grasp poses by planning in parallel.
 *
 * For each candidate grasp pose, a dedicated CcaRos planner is spawned in its
 * own thread. Each thread attempts to plan a full grasp sequence consisting of:
 *   1. A whole-body controller (WBC) approach
 *   2. An arm approach
 *   3. An arm grab
 *
 * The node returns the first affordative grasp pose, i.e. the first grasp pose for which all three plans succeed,
 * or FAILURE if no candidate pose yields a valid plan.
 *
 * Notable assumptions: Both wbc and arm planning requests contain gripper goals
 *
 * @par BT Ports
 * | Direction | Name                    | Type                                          | Description |
 * |-----------|-------------------------|-----------------------------------------------|--------------------------------------------------|
 * | Input     | wbc_approach_req        | std::shared_ptr<CcaRos::PlanningRequest>      | Planning request for the WBC
 * approach motion     | | Input     | arm_approach_req        | std::shared_ptr<CcaRos::PlanningRequest>      |
 * Planning request for the arm approach motion     | | Input     | arm_grab_req            |
 * std::shared_ptr<CcaRos::PlanningRequest>      | Planning request for the arm grab motion         | | Input     |
 * grasp_poses             | std::shared_ptr<geometry_msgs::msg::PoseArray>| Array of candidate grasp poses to evaluate
 * | | Output    | affordative_grasp_pose  | std::shared_ptr<geometry_msgs::msg::PoseStamped> | First grasp pose for
 * which all plans succeeded|
 *
 * @inherits BT::SyncActionNode
 * @inherits rclcpp::Node
 */
namespace chair_manipulation
{
class GetAffordativeGraspPose : public BT::SyncActionNode, public rclcpp::Node
{
  public:
    /**
     * @brief Construct a GetAffordativeGraspPose node.
     * @param name  The name of the node, used for both the BT node and the ROS 2 node.
     * @param config BT node configuration containing the blackboard and port mappings.
     */
    GetAffordativeGraspPose(const std::string &name, const BT::NodeConfig &config);

    /**
     * @brief Returns the list of input and output ports for this BT node.
     * @return BT::PortsList containing all declared ports.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Executes the node logic.
     *
     * Reads all input ports, spawns one planning thread per candidate grasp pose,
     * and waits for the first successful full plan or for all threads to complete.
     *
     * @return BT::NodeStatus::SUCCESS if an affordative grasp pose is found.
     * @return BT::NodeStatus::FAILURE if any input port is missing or no valid plan is found.
     */
    BT::NodeStatus tick() override;

  private:
    const std::chrono::milliseconds timeout_{
        100}; ///< Maximum time to wait for planning threads to find an affordative grasp pose
    static constexpr int arm_start_index_in_wbc_traj =
        3; ///< Starting index of arm joints in the whole-body trajectory (first three are for the base)
    static constexpr int arm_num_joints = 6; ///< Number of arm joints
};

} // namespace chair_manipulation

#endif // GET_AFFORDATIVE_GRASP_POSE_HPP
