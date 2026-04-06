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

 // Set canonical_pose_from base for all planning requests
    const std::string &grasp_pose_frame_id = grasp_poses.header.frame_id;
    affordance_util::PoseFrom canonical_pose_from_base;
    canonical_pose_from_base.method = affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
    canonical_pose_from_base.frame_name = grasp_pose_frame_id;

    // Set affordance_info_from base for all planning requests
    affordance_util::ScrewInfoFrom affordance_info_from_base;
    affordance_info_from_base.method = affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
    affordance_info_from_base.frame_name = grasp_pose_frame_id;
    affordance_info_from_base.axis_in_final_pose =
        affordance_util::axis_to_vec(affordance_util::Axis::X_MINUS); // Along the outward-facing normal of the grasp

    // Synchronization primitives for first-success detection
    std::mutex result_mutex;
    std::condition_variable result_cv;
    bool found_successful_plan = false;
    size_t completed_threads = 0;
    geometry_msgs::msg::Pose affordative_grasp_pose;

    // Create a planner per grasp pose
    // NOTE: geometry_msgs::msg::Pose has no hash, so we use index-based storage
    std::vector<std::pair<geometry_msgs::msg::Pose, std::shared_ptr<cca_ros::CcaRos>>> grasp_pose_to_planners;
    for (size_t i = 0; i < grasp_poses.poses.size(); ++i)
    {
        auto planner = std::make_shared<cca_ros::CcaRos>("cca_ros_" + std::to_string(i), rclcpp::NodeOptions());
        grasp_pose_to_planners.emplace_back(grasp_poses.poses[i], planner);
    }

    const size_t total_threads = grasp_pose_to_planners.size();

    // Lambda to plan all requests for a given grasp pose and signal on first success
    auto is_grasp_pose_affordative = [&](std::stop_token stop_token, std::shared_ptr<cca_ros::CcaRos> planner,
                                         const geometry_msgs::msg::Pose &grasp_pose) {
        // Check for stop request before starting expensive planning
        if (stop_token.stop_requested())
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            completed_threads++;
            result_cv.notify_one();
            return;
        }

        // Convert pose to Eigen
        Eigen::Isometry3d grasp_pose_eigen;
        tf2::fromMsg(grasp_pose, grasp_pose_eigen);

        // Set canonical pose info
        affordance_util::PoseFrom canonical_pose_from = canonical_pose_from_base;
        canonical_pose_from.post_transform = grasp_pose_eigen.matrix();

        // Set affordance info
        affordance_util::ScrewInfoFrom affordance_info_from = affordance_info_from_base;
        affordance_info_from.post_transform = grasp_pose_eigen.matrix();

        // Plan WBC approach
        auto wbc_approach_req_l = wbc_approach_req;
        wbc_approach_req_l.execute_trajectory = false;
        wbc_approach_req_l.task_description.affordance_info_from = affordance_info_from;
        wbc_approach_req_l.task_description.canonical_pose_from = canonical_pose_from;
        rclcpp::spin_some(planner); // We wanna read current state here
        auto wbc_approach_response = planner->plan(wbc_approach_req_l);
        if (!wbc_approach_response.result.success || stop_token.stop_requested())
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            completed_threads++;
            result_cv.notify_one();
            return;
        }

        // Update arm approach request, seeding start state from WBC result
        auto arm_approach_req_l = arm_approach_req;
        arm_approach_req_l.execute_trajectory = false;
        arm_approach_req_l.task_description.affordance_info_from = affordance_info_from;
        arm_approach_req_l.task_description.canonical_pose_from = canonical_pose_from;
        const Eigen::VectorXd &wbc_traj_end_point = wbc_approach_response.result.cca_result.joint_trajectory.back();
        const Eigen::VectorXd &arm_start_state =
            wbc_traj_end_point.segment(arm_start_index_in_wbc_traj, arm_num_joints);
        const double gripper_start_state = wbc_traj_end_point(arm_start_index_in_wbc_traj + arm_num_joints);
        arm_approach_req_l.start_state.robot = arm_start_state;
        arm_approach_req_l.start_state.gripper = gripper_start_state;

        // Disable execution for arm grab request
        auto arm_grab_req_l = arm_grab_req;
        arm_grab_req_l.execute_trajectory = false;
        arm_grab_req_l.task_description.affordance_info_from = affordance_info_from;

        // Plan arm requests
        std::vector<cca_ros::PlanningRequest> arm_reqs = {arm_approach_req_l, arm_grab_req_l};
        auto arm_response = planner->plan(arm_reqs);
        if (!arm_response.result.success || stop_token.stop_requested())
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            completed_threads++;
            result_cv.notify_one();
            return;
        }

        // Signal success via condition variable
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            if (!found_successful_plan)
            { // Only set the first successful plan
                found_successful_plan = true;
                affordative_grasp_pose = grasp_pose;
            }
            completed_threads++;
            result_cv.notify_one();
        }
    };

    // Launch planning threads with stop tokens
    std::vector<std::jthread> planning_threads;
    for (const auto &[grasp_pose, planner] : grasp_pose_to_planners)
    {
        planning_threads.emplace_back([planner, grasp_pose, &is_grasp_pose_affordative](std::stop_token st) {
            is_grasp_pose_affordative(st, planner, grasp_pose);
        });
    }

    // Wait for first success, all threads to finish, or timeout
    {
        std::unique_lock<std::mutex> lock(result_mutex);
        result_cv.wait_for(lock, timeout,
                           [&]() { return found_successful_plan || completed_threads == total_threads; });
    }

    // Request stop on all threads (no-op if already finished)
    for (auto &t : planning_threads)
    {
        t.request_stop();
    }

    if (!found_successful_plan)
    {
        return std::nullopt; // No affordative grasp pose found within timeout
    }

    // Set output
    geometry_msgs::msg::PoseStamped affordative_grasp_pose_stamped;
    affordative_grasp_pose_stamped.pose = affordative_grasp_pose;
    affordative_grasp_pose_stamped.header.stamp = grasp_poses.header.stamp;
    affordative_grasp_pose_stamped.header.frame_id = grasp_pose_frame_id;

    return affordative_grasp_pose_stamped;
}

} // namespace cca_ros_features
