#include "cca_ros_features/cca_ros_features.hpp"

namespace cca_ros_features
{

std::optional<geometry_msgs::msg::PoseStamped> getAffordativeGraspPose(
    const cca_ros::PlanningRequest &wbc_approach_req,
    const cca_ros::PlanningRequest &arm_approach_req,
    const cca_ros::PlanningRequest &arm_grab_req,
    const geometry_msgs::msg::PoseArray &grasp_poses,
    std::chrono::milliseconds timeout,
    int arm_start_index_in_wbc_traj,
    int arm_num_joints)
{

    return result;
}

} // namespace cca_ros_features
