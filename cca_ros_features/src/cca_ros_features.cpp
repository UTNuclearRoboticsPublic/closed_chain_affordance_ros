#include "cca_ros_features/cca_ros_features.hpp"

namespace cca_ros_features
{

bool is_plannable(
    std::shared_ptr<cca_ros::CcaRos> planner,
    const std::vector<cca_ros::PlanningRequest> &requests)
{
    using PlanningSegment = std::vector<cca_ros::PlanningRequest>;

    // Segment requests by consecutive planning group
    std::vector<PlanningSegment> segments;
    for (const auto &req : requests)
    {
        if (segments.empty() || segments.back().front().planning_group != req.planning_group)
        {
            segments.push_back({req});
        }
        else
        {
            segments.back().push_back(req);
        }
    }

    // Lambda to extract start state for the next segment from the previous segment's trajectory end point
    auto extractStartState = [&](const cca_ros::PlanningResponse &prev_response,
                                 const std::string &next_group) -> Eigen::VectorXd {
        const std::vector<std::string> &next_joint_names = planner->getJointNames(next_group);
        const auto &prev_traj = prev_response.result.joint_trajectory.trajectory;
        const auto &last_point = prev_traj.points.back();

        // Build name->position map from previous segment's end point
        std::unordered_map<std::string, double> name_to_pos;
        for (size_t i = 0; i < prev_traj.joint_names.size(); ++i)
        {
            name_to_pos[prev_traj.joint_names[i]] = last_point.positions[i];
        }

        // Extract positions in next group's joint order
        Eigen::VectorXd start_state(next_joint_names.size());
        for (size_t i = 0; i < next_joint_names.size(); ++i)
        {
            start_state[i] = name_to_pos.at(next_joint_names[i]);
        }
        return start_state;
    };

    cca_ros::PlanningResponse prev_response;

    for (size_t seg_idx = 0; seg_idx < segments.size(); ++seg_idx)
    {
        auto &segment = segments[seg_idx];

        // Chain start state from previous segment's trajectory end point
        if (seg_idx > 0)
        {
            segment.front().start_state.robot =
                extractStartState(prev_response, segment.front().planning_group);
        }

        // Plan this segment
        const auto response = planner->plan(segment);
        if (!response.result.success)
        {
            return false;
        }
        prev_response = response;
    }

    return true;
}

std::optional<geometry_msgs::msg::PoseStamped> get_affordative_grasp_pose(
    std::shared_ptr<rclcpp::Node> node,
    const std::vector<cca_ros::PlanningRequest> &approach_reqs,
    const cca_ros::PlanningRequest &grab_req,
    const geometry_msgs::msg::PoseArray &grasp_poses,
    std::chrono::milliseconds timeout)
{
    // Verify approach_reqs are indeed approach types
    for (const auto &req : approach_reqs)
    {
        if (req.task_description.motion_type != cc_affordance_planner::MotionType::APPROACH)
        {
            RCLCPP_ERROR(node->get_logger(),
                "All approach_reqs must have task_description.motion_type == cc_affordance_planner::MotionType::APPROACH");
            return std::nullopt;
        }
    }

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
        auto planner_node = std::make_shared<rclcpp::Node>("cca_ros_node_" + std::to_string(i));
        auto planner = std::make_shared<cca_ros::CcaRos>(planner_node);
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

        // Set affordance info -- affordance is defined relative to the grasp pose
        affordance_util::ScrewInfoFrom affordance_info_from = affordance_info_from_base;
        affordance_info_from.post_transform = grasp_pose_eigen.matrix();

        // Fill in grasp pose info for all approach requests
        std::vector<cca_ros::PlanningRequest> reqs;
        for (const auto &req : approach_reqs)
        {
            auto req_l = req;
            req_l.execute_trajectory = false;
            req_l.task_description.affordance_info_from = affordance_info_from;
            req_l.task_description.canonical_pose_from = canonical_pose_from;
            reqs.push_back(req_l);
        }

        // Fill in affordance info for grab request (grab affordance is defined relative to the grasp pose)
        auto grab_req_l = grab_req;
        grab_req_l.execute_trajectory = false;
        grab_req_l.task_description.affordance_info_from = affordance_info_from;
        reqs.push_back(grab_req_l);

        // Check if these requests are plannable with this grasp pose
        if (!is_plannable(planner, reqs) || stop_token.stop_requested())
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
            {
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
        return std::nullopt;
    }

    // Build and return the stamped pose
    geometry_msgs::msg::PoseStamped affordative_grasp_pose_stamped;
    affordative_grasp_pose_stamped.pose = affordative_grasp_pose;
    affordative_grasp_pose_stamped.header.stamp = grasp_poses.header.stamp;
    affordative_grasp_pose_stamped.header.frame_id = grasp_pose_frame_id;

    return affordative_grasp_pose_stamped;
}

} // namespace cca_ros_features
