///////////////////////////////////////////////////////////////////////////////
//      Title     : cca_ros_features.hpp
//      Project   : cca_ros_features
//      Created   : 2026
//      Author    : Crasun Jans
///////////////////////////////////////////////////////////////////////////////

#ifndef CCA_ROS_FEATURES_HPP_
#define CCA_ROS_FEATURES_HPP_

#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <unordered_map>

namespace cca_ros_features
{

/**
 * @brief Plans a sequence of requests spanning multiple planning groups
 * sequentially. Requests with the same consecutive planning group are batched
 * into one plan() call. Start state is chained between segments using joint
 * names from the previous segment's trajectory end point.
 *
 * Note: The caller is responsible for spinning the underlying ROS node.
 *
 * @param planner Shared pointer to the CcaRos planner.
 * @param requests Sequence of planning requests, potentially spanning multiple planning groups.
 * @param stop_token Optional stop token to allow cooperative cancellation of planning. 
 * @return true if all segments planned successfully, false otherwise.
 */
bool is_plannable(
    std::shared_ptr<cca_ros::CcaRos> planner,
    const std::vector<cca_ros::PlanningRequest> &requests, std::stop_token stop_token = std::stop_token{});

/**
 * @brief Finds the first affordative grasp pose from a set of candidates by
 * planning in parallel. For each candidate grasp pose, a dedicated CcaRos
 * planner is spawned in its own thread and attempts to plan a full sequence
 * of approach requests followed by a grab request.
 *
 * Note: The caller is responsible for spinning the underlying ROS nodes.
 *
 * @param context Shared CcaRosContext used for ROS infrastructure (logging, TF, joint states).
 * @param approach_reqs Sequence of planning requests of type APPROACH, potentially spanning multiple planning groups.
 * @param grab_req Planning request for the grab motion.
 * @param grasp_poses Array of candidate grasp poses to evaluate.
 * @param timeout Maximum time to wait across all planning threads.
 * @return The first affordative grasp pose stamped, or std::nullopt if none found.
 */
std::optional<geometry_msgs::msg::PoseStamped> get_affordative_grasp_pose(
    std::shared_ptr<cca_ros::CcaRosContext> context,
    const std::vector<cca_ros::PlanningRequest> &approach_reqs,
    const cca_ros::PlanningRequest &grab_req,
    const geometry_msgs::msg::PoseArray &grasp_poses,
    std::chrono::milliseconds timeout);

} // namespace cca_ros_features

#endif // CCA_ROS_FEATURES_HPP_
