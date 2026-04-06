///////////////////////////////////////////////////////////////////////////////
//      Title     : cca_ros_features.hpp
//      Project   : cca_ros_features
//      Created   : 2026
//      Author    : Crasun Jans
///////////////////////////////////////////////////////////////////////////////

#ifndef CCA_ROS_FEATURES_HPP_
#define CCA_ROS_FEATURES_HPP_

#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>
#include <tf2_eigen/tf2_eigen.hpp>

namespace cca_ros_features
{

/**
 * @brief Finds the first affordative grasp pose from a set of candidates by
 * planning in parallel. For each candidate grasp pose, a dedicated CcaRos
 * planner is spawned in its own thread and attempts to plan a full grasp
 * sequence: a WBC approach, an arm approach (seeded from the WBC end state),
 * and an arm grab.
 *
 * @param wbc_approach_req Planning request for the WBC approach motion.
 * @param arm_approach_req Planning request for the arm approach motion.
 * @param arm_grab_req Planning request for the arm grab motion.
 * @param grasp_poses Array of candidate grasp poses to evaluate.
 * @param timeout Maximum time to wait across all planning threads.
 * @param arm_start_index_in_wbc_traj Starting index of arm joints in the WBC trajectory.
 * @param arm_num_joints Number of arm joints.
 * @return The first affordative grasp pose stamped, or std::nullopt if none found.
 */
std::optional<geometry_msgs::msg::PoseStamped> getAffordativeGraspPose(
    const cca_ros::PlanningRequest &wbc_approach_req,
    const cca_ros::PlanningRequest &arm_approach_req,
    const cca_ros::PlanningRequest &arm_grab_req,
    const geometry_msgs::msg::PoseArray &grasp_poses,
    std::chrono::milliseconds timeout,
    int arm_start_index_in_wbc_traj,
    int arm_num_joints);

} // namespace cca_ros_features

#endif // CCA_ROS_FEATURES_HPP_
