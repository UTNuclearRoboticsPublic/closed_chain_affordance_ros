#include <affordance_util_ros/affordance_util_ros.hpp>
namespace AffordanceUtilROS {

// Returns absolute path to the specified relative path
std::string get_abs_path_to_rel_dir(const std::string &full_path,
                                    const std::string &relative_path) {
  /* std::string full_path = __FILE__; */
  /* relative_path = "/../data/"; */
  std::filesystem::path source_file_path(full_path);
  std::filesystem::path source_file_parent_path =
      source_file_path.parent_path(); // directory containing the source file
  const std::string output = source_file_parent_path.string() + relative_path;
  return output;
}

std::vector<JointTrajPoint>
get_ordered_joint_traj(const trajectory_msgs::JointTrajectory &traj,
                       const std::vector<std::string> &joint_name_order) {

  std::vector<JointTrajPoint> ordered_traj;

  size_t nof_joints =
      joint_name_order
          .size(); // this is the relevant number of joints. Even if the
                   // trajectory contains more joints, we'll only extract
                   // relevant joint positions and in order

  // First, we determine the order that the indices need to be in to match
  // the joint_name_order.
  //  Store unordered joint names in a map and assign indices
  std::unordered_map<std::string, size_t> joint_index_map;
  for (size_t i = 0; i < traj.joint_names.size(); ++i) {
    joint_index_map[traj.joint_names[i]] = i;
  }

  // Determine the correct index order of the unordered joint names to match
  // ordered joint names
  std::vector<size_t> index_order(nof_joints);
  for (size_t i = 0; i < nof_joints; ++i) {
    const std::string &joint_name = joint_name_order[i];

    auto it = joint_index_map.find(joint_name);
    if (it != joint_index_map.end()) {
      index_order[i] = it->second;
    } else {
      // Handle the case where the joint name is not found
      std::cerr << "Joint name not found: " << joint_name << std::endl;
    }
  }

  // Iterate through each trajectory point and extract joint states in the
  // correct order as well as timestamps
  for (const auto &point : traj.points) {
    JointTrajPoint ordered_traj_point;
    ordered_traj_point.positions.conservativeResize(nof_joints);

    // Extract the joint positions in correct order and fill it in the
    // result
    for (size_t i = 0; i < joint_name_order.size(); ++i) {
      ordered_traj_point.positions[i] = point.positions[index_order[i]];
    }

    // Retrieve timestamp as well and push it into the result vector
    ordered_traj_point.timestamp = point.time_from_start.toSec();
    ordered_traj.push_back(ordered_traj_point);
  }

  return ordered_traj;
}

JointTrajPoint
get_ordered_joint_states(const sensor_msgs::JointState::ConstPtr &joint_states,
                         const std::vector<std::string> &joint_name_order) {

  size_t nof_joints =
      joint_name_order
          .size(); // this is the relevant number of joints. Even if the
                   // trajectory contains more joints, we'll only extract
                   // relevant joint positions and in order
  JointTrajPoint ordered_joint_states;
  ordered_joint_states.positions.conservativeResize(nof_joints);

  // First, we determine the order that the indices need to be in to match
  // the joint_name_order.
  //  Store unordered joint names in a map and assign indices
  // Create a mapping between joint names and indices
  std::unordered_map<std::string, size_t> joint_index_map;
  for (size_t i = 0; i < joint_states->name.size(); ++i) {
    joint_index_map[joint_states->name[i]] = i;
  }

  // Determine the index order
  std::vector<size_t> index_order(nof_joints);
  for (size_t i = 0; i < nof_joints; ++i) {
    const std::string &joint_name = joint_name_order[i];
    auto it = joint_index_map.find(joint_name);
    if (it != joint_index_map.end()) {
      index_order[i] = it->second;
    } else {
      // Handle the case where the joint name is not found
      std::cerr << "Joint name not found: " << joint_name << std::endl;
    }
  }

  // Extract the joint positions in correct order
  for (size_t i = 0; i < nof_joints; ++i) {
    ordered_joint_states.positions[i] = joint_states->position[index_order[i]];
  }

  // Extract and set the timestamp as well
  ordered_joint_states.timestamp = joint_states->header.stamp.toSec();

  return ordered_joint_states;
}

Eigen::Isometry3d get_htm(const std::string &target_frame,
                          const std::string &reference_frame,
                          tf2_ros::Buffer &tf_buffer) {

  tf2_ros::TransformListener tf_listener(
      tf_buffer); // Initialize listener with the buffer

  Eigen::Isometry3d htm; // Output

  geometry_msgs::TransformStamped
      transform_stamped; // ros message to hold transform info

  // Query the listener for the desired transformation at the latest
  // available time

  try {
    transform_stamped = tf_buffer.lookupTransform(
        target_frame, reference_frame, ros::Time(0), ros::Duration(0.3));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
  }

  // Convert the message to Eigen::Isometry3d type
  // Extract translation and rotation from the TransformStamped message
  Eigen::Vector3d translation(transform_stamped.transform.translation.x,
                              transform_stamped.transform.translation.y,
                              transform_stamped.transform.translation.z);
  Eigen::Quaterniond rotation(transform_stamped.transform.rotation.w,
                              transform_stamped.transform.rotation.x,
                              transform_stamped.transform.rotation.y,
                              transform_stamped.transform.rotation.z);

  // Set the translation and rotation components of the HTM
  htm.translation() = translation;
  htm.linear() = rotation.toRotationMatrix();

  return htm;
}

control_msgs::FollowJointTrajectoryGoal follow_joint_trajectory_msg_builder(
    const std::vector<Eigen::VectorXd> &bare_trajectory,
    const std::vector<double> &config_offset,
    const std::vector<std::string> &joint_names, const double &time_step) {
  control_msgs::FollowJointTrajectoryGoal fjtg_msg;

  // Set the joint names
  fjtg_msg.trajectory.joint_names = joint_names;

  fjtg_msg.trajectory.points.resize(
      bare_trajectory.size()); // resize points array before filling

  size_t j = 0;
  for (const Eigen::VectorXd &point : bare_trajectory) {
    trajectory_msgs::JointTrajectoryPoint &trajectory_point =
        fjtg_msg.trajectory.points[j];

    // Fill out point times
    trajectory_point.time_from_start = ros::Duration(j * time_step);

    // Fill out position values
    trajectory_point.positions.resize(
        joint_names.size()); // resize positions array before filling
    for (size_t i = 0; i < joint_names.size(); i++) {
      trajectory_point.positions[i] = point[i] + config_offset[i];
    }

    j++;
  }

  return fjtg_msg;
}
} // namespace AffordanceUtilROS