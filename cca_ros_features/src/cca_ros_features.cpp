#include "cca_ros_features/cca_ros_features.hpp"

namespace cca_ros_features
{

bool is_plannable(
    std::shared_ptr<cca_ros::CcaRos> planner,
    const std::vector<cca_ros::PlanningRequest> &requests, std::stop_token stop_token)
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
        const std::vector<std::string> &next_joint_names = planner->get_joint_names(next_group);
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

	// Check for stop request before planning each segment
	if (stop_token.stop_requested())
	{
	    return false;
	}

        // Plan this segment
        const auto response = planner->plan(segment); // Note: plan() is blocking until a response is received. In the future, it may be desirable to have a plan_async() function in CcaRos that accepts a stop token, or have another mechanism to immediately interrupt. For now, since CCA planning is super quick anyways, this works for all practical purposes.
        if (!response.result.success)
        {
            return false;
        }
        prev_response = response;
    }

    return true;
}

std::optional<geometry_msgs::msg::PoseStamped> get_affordative_grasp_pose(
    std::shared_ptr<cca_ros::CcaRosContext> context,
    const std::vector<cca_ros::PlanningRequest> &approach_reqs,
    const cca_ros::PlanningRequest &grab_req,
    const geometry_msgs::msg::PoseArray &grasp_poses,
    std::chrono::milliseconds timeout,
    std::stop_token stop_token)
{

    auto node_logger = context->get_node()->get_logger();

    // Verify approach_reqs are indeed approach types
    for (const auto &req : approach_reqs)
    {
        if (req.task_description.motion_type != cc_affordance_planner::MotionType::APPROACH)
        {
            RCLCPP_ERROR(node_logger,
                "All approach_reqs must have task_description.motion_type == cc_affordance_planner::MotionType::APPROACH");
            return std::nullopt;
        }
        // Ensure approach reqs have canonical_pose_from and affordance_info_from set to FROM_FRAME_NAME
        if (req.task_description.canonical_pose_from.method != affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME ||
            req.task_description.affordance_info_from.method != affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME)
        {
            RCLCPP_ERROR(node_logger,
                "All approach_reqs must have task_description.canonical_pose_from and task_description.affordance_info_from set to method == FROM_FRAME_NAME");
            return std::nullopt;
        }
    }

    // Verify grab_req is indeed an affordance type
    if (grab_req.task_description.motion_type != cc_affordance_planner::MotionType::AFFORDANCE)
    {
        RCLCPP_ERROR(node_logger,
            "grab_req must have task_description.motion_type == cc_affordance_planner::MotionType::AFFORDANCE");
        return std::nullopt;
    }

    // Ensure grab req has affordance_info_from set to FROM_FRAME_NAME
    if (grab_req.task_description.affordance_info_from.method != affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME)
    {
        RCLCPP_ERROR(node_logger,
            "grab_req must have task_description.affordance_info_from set to method == FROM_FRAME_NAME");
        return std::nullopt;
    }

    // Ensure affordance_info_from has axis_in_final_pose set
    if (grab_req.task_description.affordance_info_from.axis_in_final_pose.hasNaN())
    {
        RCLCPP_ERROR(node_logger,
            "grab_req must have task_description.affordance_info_from.axis_in_final_pose set to a valid axis. Affordance axis is defined relative to the grasp pose.");
        return std::nullopt;
    }

    if (stop_token.stop_requested())
    {
        return std::nullopt;
    }

    // Extract grasp pose frame id
    const std::string &grasp_pose_frame_id = grasp_poses.header.frame_id;

    // Synchronization primitives for first-success detection
    std::mutex result_mutex;
    std::condition_variable result_cv;
    bool found_successful_plan = false;
    size_t completed_threads = 0;
    geometry_msgs::msg::Pose affordative_grasp_pose;

    // Create a planner per grasp pose, all sharing the same context
    using PlannerEntry = std::pair<geometry_msgs::msg::Pose, std::shared_ptr<cca_ros::CcaRos>>;
    std::vector<PlannerEntry> grasp_pose_to_planners;
    for (size_t i = 0; i < grasp_poses.poses.size(); ++i)
    {
        auto planner = std::make_shared<cca_ros::CcaRos>(context);
        grasp_pose_to_planners.emplace_back(grasp_poses.poses[i], planner);
    }

    const size_t total_threads = grasp_pose_to_planners.size();

    // Lambda to plan all requests for a given grasp pose and signal on first success
    auto is_grasp_pose_affordative = [&](std::stop_token st, std::shared_ptr<cca_ros::CcaRos> planner,
                                         const geometry_msgs::msg::Pose &grasp_pose) {
        // Check for stop request before starting expensive planning
        if (st.stop_requested())
        {
            std::lock_guard<std::mutex> lock(result_mutex);
            completed_threads++;
            result_cv.notify_one();
            return;
        }

        // Convert pose to Eigen
        Eigen::Isometry3d grasp_pose_eigen;
        tf2::fromMsg(grasp_pose, grasp_pose_eigen);

        // Fill in grasp pose info for all approach requests
        std::vector<cca_ros::PlanningRequest> reqs;
        for (const auto &req : approach_reqs)
        {
            auto req_l = req;
            req_l.execute_trajectory = false;
            req_l.execute_partial_trajectory = false;
            req_l.visualize_trajectory = false;
            req_l.task_description.affordance_info_from.frame_name = grasp_pose_frame_id;
            req_l.task_description.affordance_info_from.post_transform = grasp_pose_eigen.matrix();
            req_l.task_description.canonical_pose_from.frame_name = grasp_pose_frame_id;
            req_l.task_description.canonical_pose_from.post_transform = grasp_pose_eigen.matrix();
            reqs.push_back(req_l);
        }

        // Fill in affordance info for grab request (grab affordance is defined relative to the grasp pose)
        auto grab_req_l = grab_req;
        grab_req_l.execute_trajectory = false;
        grab_req_l.execute_partial_trajectory = false;
        grab_req_l.visualize_trajectory = false;
        grab_req_l.task_description.affordance_info_from.frame_name = grasp_pose_frame_id;
        grab_req_l.task_description.affordance_info_from.post_transform = grasp_pose_eigen.matrix();
        reqs.push_back(grab_req_l);

        // Check if these requests are plannable with this grasp pose
        if (!is_plannable(planner, reqs, st))
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
        planning_threads.emplace_back([planner, grasp_pose, &is_grasp_pose_affordative, stop_token](std::stop_token jthread_st) {
            if (stop_token.stop_requested()){return;}
            is_grasp_pose_affordative(jthread_st, planner, grasp_pose);
        });
    }

    // Wait for first success, all threads to finish, or timeout
    {
        std::unique_lock<std::mutex> lock(result_mutex);
        result_cv.wait_for(lock, timeout,
                           [&]() { return found_successful_plan || completed_threads == total_threads || stop_token.stop_requested(); });
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
