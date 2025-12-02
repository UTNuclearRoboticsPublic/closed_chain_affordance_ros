#include <ros_cpp_util/ros_cpp_util.hpp>
namespace ros_cpp_util
{

CustomException::CustomException(const char *message) : msg(message) {}

// Override the what() function to provide a description of the exception
const char *CustomException::what() const noexcept { return msg.c_str(); }

std::string get_filepath_inside_pkg(const std::string &package_name, const std::string &rel_dir,
                                    const std::string &filename)
{
    try
    {
        std::string full_filepath; // output of the function

        // Get the path to the package
        const std::string package_path = ament_index_cpp::get_package_share_directory(package_name);

        // Build full filepath and check for errors
        if (!package_path.empty())
        {
            full_filepath = package_path + rel_dir + filename;
        }
        else
        {
            throw CustomException(("Failed to find path for package '" + package_name).c_str());
        }

        // Make sure the file exists
        if (!std::filesystem::exists(full_filepath))
        {
            throw CustomException(("File does not exist at this path: '" + full_filepath).c_str());
        }

        return full_filepath;
    }
    // throw runtime error if exception is caught
    catch (const CustomException &e)
    {
        std::cerr << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error("Cannot proceed without building a valid filepath to the ROS package file");
    }
}

std::string get_abs_path_to_rel_dir(const std::string &full_path, const std::string &relative_path)
{
    std::filesystem::path source_file_path(full_path);
    std::filesystem::path source_file_parent_path =
        source_file_path.parent_path();                                          // Directory containing the source file
    const std::string output = source_file_parent_path.string() + relative_path; // Full path to relative directory

    return output;
}

std::vector<JointTrajPoint> get_ordered_joint_traj(const trajectory_msgs::msg::JointTrajectory &traj,
                                                   const std::vector<std::string> &joint_name_order)
{

    std::vector<JointTrajPoint> ordered_traj;

    const size_t nof_joints = joint_name_order.size(); // This is the relevant number of joints. Even if the
                                                       // trajectory contains more joints, we'll only extract
                                                       // relevant joint positions and in order

    // First, we determine the order that the indices need to be in to match
    // the joint_name_order.
    //  Store unordered joint names in a map and assign indices
    std::unordered_map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
        joint_index_map[traj.joint_names[i]] = i;
    }

    // Determine the correct index order of the unordered joint names to match
    // ordered joint names
    std::vector<size_t> index_order(nof_joints);
    for (size_t i = 0; i < nof_joints; ++i)
    {
        const std::string &joint_name = joint_name_order[i];

        auto it = joint_index_map.find(joint_name);
        if (it != joint_index_map.end())
        {
            index_order[i] = it->second;
        }
        else
        {
            // Handle the case where the joint name is not found
            std::cerr << "Joint name not found: " << joint_name << std::endl;
        }
    }

    // Iterate through each trajectory point and extract joint states in the
    // correct order as well as timestamps
    for (const auto &point : traj.points)
    {
        JointTrajPoint ordered_traj_point;
        ordered_traj_point.positions.conservativeResize(nof_joints);

        // Extract the joint positions in correct order and fill it in the
        // result
        for (size_t i = 0; i < joint_name_order.size(); ++i)
        {
            ordered_traj_point.positions[i] = point.positions[index_order[i]];
        }

        // Retrieve timestamp as well and push it into the result vector
        ordered_traj_point.timestamp = point.time_from_start.sec;
        ordered_traj.push_back(ordered_traj_point);
    }

    return ordered_traj;
}

JointTrajPoint get_ordered_joint_states(const sensor_msgs::msg::JointState::ConstSharedPtr &joint_states,
                                        const std::vector<std::string> &joint_name_order)
{

    const size_t nof_joints = joint_name_order.size(); // This is the relevant number of joints. Even if the
                                                       // trajectory contains more joints, we'll only extract
                                                       // relevant joint positions and in order
    JointTrajPoint ordered_joint_states;
    ordered_joint_states.positions.conservativeResize(nof_joints);

    // First, we determine the order that the indices need to be in to match
    // the joint_name_order.
    //  Store unordered joint names in a map and assign indices
    // Create a mapping between joint names and indices
    std::unordered_map<std::string, size_t> joint_index_map;
    for (size_t i = 0; i < joint_states->name.size(); ++i)
    {
        joint_index_map[joint_states->name[i]] = i;
    }

    // Determine the index order
    std::vector<size_t> index_order(nof_joints);
    for (size_t i = 0; i < nof_joints; ++i)
    {
        const std::string &joint_name = joint_name_order[i];
        auto it = joint_index_map.find(joint_name);
        if (it != joint_index_map.end())
        {
            index_order[i] = it->second;
        }
        else
        {
            // Handle the case where the joint name is not found
            std::cerr << "Joint name not found: " << joint_name << std::endl;
        }
    }

    // Extract the joint positions in correct order
    for (size_t i = 0; i < nof_joints; ++i)
    {
        ordered_joint_states.positions[i] = joint_states->position[index_order[i]];
    }

    // Extract and set the timestamp as well
    ordered_joint_states.timestamp = joint_states->header.stamp.sec;
    // Extract and set the timestamp as well (combine seconds and nanoseconds)
    ordered_joint_states.timestamp = 
        static_cast<uint64_t>(joint_states->header.stamp.sec) * 1000000000ULL + 
        static_cast<uint64_t>(joint_states->header.stamp.nanosec);

    return ordered_joint_states;
}

control_msgs::action::FollowJointTrajectory_Goal follow_joint_trajectory_msg_builder(
    const std::vector<Eigen::VectorXd> &bare_trajectory, const Eigen::VectorXd &config_offset,
    const std::vector<std::string> &joint_names, const double &time_step)
{
    control_msgs::action::FollowJointTrajectory_Goal fjtg_msg;

    const double tolerance = 1e-6; // tolerace for zero

    // Set the joint names
    fjtg_msg.trajectory.joint_names = joint_names;

    if (config_offset.isMuchSmallerThan(tolerance)) // If offset is 0 then, just fill out message without adjustments
    {
        fjtg_msg.trajectory.points.resize(bare_trajectory.size());
        // Fill out the trajectory with bare_trajectory
        size_t j = 0;
        for (const Eigen::VectorXd &point : bare_trajectory)
        {
            trajectory_msgs::msg::JointTrajectoryPoint &trajectory_point = fjtg_msg.trajectory.points[j];

            // Fill out point times
            trajectory_point.time_from_start = rclcpp::Duration::from_seconds(j * time_step);

            // Fill out position values
            trajectory_point.positions.resize(joint_names.size()); // resize positions array before filling
            for (size_t i = 0; i < joint_names.size(); i++)
            {
                trajectory_point.positions[i] = point[i];
            }

            j++;
        }
    }
    else // If offset is non-zero then, make offset the first point and the rest shifted by the offset. Time starts from
         // the first point.
    {
        fjtg_msg.trajectory.points.resize(bare_trajectory.size() +
                                          1); // Resize points array before filling to bare_trajectory.size()+1
                                              // since the first point will be the config_offset itself
        // Add the first point with config_offset
        trajectory_msgs::msg::JointTrajectoryPoint &first_point = fjtg_msg.trajectory.points[0];
        first_point.time_from_start = rclcpp::Duration::from_seconds(0.0);
        // Start time is 0
        first_point.positions.resize(joint_names.size());
        for (size_t i = 0; i < joint_names.size(); i++)
        {
            first_point.positions[i] = config_offset[i];
        }

        // Fill out the rest of the trajectory with bare_trajectory
        size_t j = 1;
        for (const Eigen::VectorXd &point : bare_trajectory)
        {
            trajectory_msgs::msg::JointTrajectoryPoint &trajectory_point = fjtg_msg.trajectory.points[j];

            // Fill out point times
            trajectory_point.time_from_start = rclcpp::Duration::from_seconds(j * time_step);

            // Fill out position values
            trajectory_point.positions.resize(joint_names.size()); // resize positions array before filling
            for (size_t i = 0; i < joint_names.size(); i++)
            {
                trajectory_point.positions[i] = point[i] + config_offset[i];
            }

            j++;
        }
    }

    return fjtg_msg;
}

trajectory_msgs::msg::JointTrajectory stitch_trajectories(const std::vector<trajectory_msgs::msg::JointTrajectory> &trajectories)
{
    trajectory_msgs::msg::JointTrajectory stitched_traj;
    if (trajectories.empty())
        return stitched_traj;
    
    // Copy joint names from first trajectory
    stitched_traj.joint_names = trajectories.front().joint_names;
    
    // Time offset for the first trajectory is 0
    rclcpp::Duration time_offset = rclcpp::Duration::from_seconds(0.0);
    
    for (size_t traj_idx = 0; traj_idx < trajectories.size(); ++traj_idx)
    {
        const auto& traj = trajectories[traj_idx];
        
        // Ensure joint names match
        if (traj.joint_names != stitched_traj.joint_names)
        {
            throw std::runtime_error("Joint names mismatch during trajectory stitching");
        }
        
        // Skip empty trajectories
        if (traj.points.empty())
            continue;
        
        // For trajectories after the first, validate continuity
        if (traj_idx > 0)
        {
            const auto& prev_traj = trajectories[traj_idx - 1];
            if (!prev_traj.points.empty())
            {
                const auto& last_point = prev_traj.points.back();
                const auto& first_point = traj.points.front();
                
                // Check positions match
                if (last_point.positions.size() != first_point.positions.size())
                {
                    throw std::runtime_error("Trajectory stitching failed: position size mismatch at boundary between trajectory " 
                                           + std::to_string(traj_idx - 1) + " and " + std::to_string(traj_idx));
                }
                
                constexpr double position_tolerance = 1e-6;
                for (size_t j = 0; j < last_point.positions.size(); ++j)
                {
                    if (std::abs(last_point.positions[j] - first_point.positions[j]) > position_tolerance)
                    {
                        throw std::runtime_error("Trajectory stitching failed: position discontinuity at boundary between trajectory " 
                                               + std::to_string(traj_idx - 1) + " and " + std::to_string(traj_idx) 
                                               + " (joint " + std::to_string(j) + ": " 
                                               + std::to_string(last_point.positions[j]) + " vs " + std::to_string(first_point.positions[j]) + ")");
                    }
                }
                
                // Check velocities match (if provided)
                if (!last_point.velocities.empty() && !first_point.velocities.empty())
                {
                    if (last_point.velocities.size() != first_point.velocities.size())
                    {
                        throw std::runtime_error("Trajectory stitching failed: velocity size mismatch at boundary between trajectory " 
                                               + std::to_string(traj_idx - 1) + " and " + std::to_string(traj_idx));
                    }
                    
                    constexpr double velocity_tolerance = 1e-4;
                    for (size_t j = 0; j < last_point.velocities.size(); ++j)
                    {
                        if (std::abs(last_point.velocities[j] - first_point.velocities[j]) > velocity_tolerance)
                        {
                            throw std::runtime_error("Trajectory stitching failed: velocity discontinuity at boundary between trajectory " 
                                                   + std::to_string(traj_idx - 1) + " and " + std::to_string(traj_idx) 
                                                   + " (joint " + std::to_string(j) + ": " 
                                                   + std::to_string(last_point.velocities[j]) + " vs " + std::to_string(first_point.velocities[j]) + ")");
                        }
                    }
                }
                
                // Check accelerations match (if provided)
                if (!last_point.accelerations.empty() && !first_point.accelerations.empty())
                {
                    if (last_point.accelerations.size() != first_point.accelerations.size())
                    {
                        throw std::runtime_error("Trajectory stitching failed: acceleration size mismatch at boundary between trajectory " 
                                               + std::to_string(traj_idx - 1) + " and " + std::to_string(traj_idx));
                    }
                    
                    constexpr double acceleration_tolerance = 1e-3;
                    for (size_t j = 0; j < last_point.accelerations.size(); ++j)
                    {
                        if (std::abs(last_point.accelerations[j] - first_point.accelerations[j]) > acceleration_tolerance)
                        {
                            throw std::runtime_error("Trajectory stitching failed: acceleration discontinuity at boundary between trajectory " 
                                                   + std::to_string(traj_idx - 1) + " and " + std::to_string(traj_idx) 
                                                   + " (joint " + std::to_string(j) + ": " 
                                                   + std::to_string(last_point.accelerations[j]) + " vs " + std::to_string(first_point.accelerations[j]) + ")");
                        }
                    }
                }
            }
        }
        
        // For trajectories after the first, skip the first point (duplicate of last point from previous trajectory)
        size_t start_idx = (traj_idx == 0) ? 0 : 1;
        
        // Stitch joint trajectories with continuous timing
        for (size_t i = start_idx; i < traj.points.size(); ++i)
        {
            const auto &p = traj.points[i];
            trajectory_msgs::msg::JointTrajectoryPoint shifted_pt = p;
            rclcpp::Duration original_time(p.time_from_start);
            shifted_pt.time_from_start = rclcpp::Duration(original_time + time_offset);
            stitched_traj.points.push_back(std::move(shifted_pt));
        }
        
        // Update time offset for the next trajectory (CUMULATIVE)
        if (!traj.points.empty())
        {
            rclcpp::Duration last_point_time(traj.points.back().time_from_start);
            time_offset = rclcpp::Duration(time_offset + last_point_time);
        }
    }
    
    return stitched_traj;
}

std::string get_required_str_param(rclcpp::Node* node, const std::string& key)
{
     // Declare only if not already declared
     if (!node->has_parameter(key)) {
       (void)node->declare_parameter(key, rclcpp::ParameterType::PARAMETER_STRING);
     }

     // Now retrieve
     std::string value;
     const bool got = node->get_parameter(key, value);  
     if (got && !value.empty()) {
       return value;
     }
     log_and_throw_param_retrieval_failure(node, key, "string", got);
}

std::vector<std::string> get_required_str_array_param(rclcpp::Node* node, const std::string& key)
{
     // Declare only if not already declared
     if (!node->has_parameter(key)) {
       (void)node->declare_parameter(key, rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
     }

     // Now retrieve
     std::vector<std::string> value;
     const bool got = node->get_parameter(key, value);  
     if (got && !value.empty()) {
       return value;
     }
     log_and_throw_param_retrieval_failure(node, key, "string array", got);
}

std::vector<double> get_required_double_array_param(rclcpp::Node* node, const std::string& key)
{
     // Declare only if not already declared
     if (!node->has_parameter(key)) {
       (void)node->declare_parameter(key, rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);
     }

     // Now retrieve
     std::vector<double> value;
     const bool got = node->get_parameter(key, value);  
     if (got && !value.empty()) {
       return value;
     }
     log_and_throw_param_retrieval_failure(node, key, "double array", got);
}

} // namespace ros_cpp_util

namespace {

void log_and_throw_param_retrieval_failure(const rclcpp::Node* node, const std::string& key, const std::string& expected_type, bool got) {
    std::ostringstream oss;
    oss << "Required parameter '" << key << "' is "
     << (got ? "empty" : "not set or wrong type (expected " + expected_type + ")");
    RCLCPP_FATAL(node->get_logger(), "%s", oss.str().c_str());
    throw std::runtime_error(oss.str());
}

}

