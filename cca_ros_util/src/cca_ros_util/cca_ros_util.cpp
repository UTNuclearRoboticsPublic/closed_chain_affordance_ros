#include "cca_ros_util/cca_ros_util.hpp"
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <sstream>

namespace cca_ros_util
{

// Converts a cca_ros_msgs::msg::PlanningRequest to a cca_ros PlanningRequest
cca_ros::PlanningRequest convert_cca_ros_action_to_req(const cca_ros_msgs::msg::PlanningRequest &msg)
{
    cca_ros::PlanningRequest req;

    // Convert planning group
    req.planning_group = msg.planning_group;

    // Convert Planner Config
    req.planner_config.accuracy = msg.planner_config.accuracy;
    req.planner_config.closure_err_threshold_ang = msg.planner_config.closure_err_threshold_ang;
    req.planner_config.closure_err_threshold_lin = msg.planner_config.closure_err_threshold_lin;
    req.planner_config.ik_max_itr = msg.planner_config.ik_max_itr;
    req.planner_config.update_method = update_method_from_msg(msg.planner_config.update_method.value);

    // Convert Task Description
    req.task_description.affordance_info.type = screw_type_from_msg(msg.task_description.affordance_info.type.value);

    // Convert axis, location, screw vector to Eigen::VectorXd
    req.task_description.affordance_info.axis = Eigen::VectorXd::Map(msg.task_description.affordance_info.axis.data(),
                                                                     msg.task_description.affordance_info.axis.size());
    req.task_description.affordance_info.location = Eigen::VectorXd::Map(
        msg.task_description.affordance_info.location.data(), msg.task_description.affordance_info.location.size());
    req.task_description.affordance_info.screw = Eigen::VectorXd::Map(
        msg.task_description.affordance_info.screw.data(), msg.task_description.affordance_info.screw.size());

    // Copy other fields
    req.task_description.affordance_info.pitch = msg.task_description.affordance_info.pitch;

    // Convert affordance_info_from field
    req.task_description.affordance_info_from.method =
        pose_specification_method_from_msg(msg.task_description.affordance_info_from.method.value);
    req.task_description.affordance_info_from.frame_name = msg.task_description.affordance_info_from.frame_name;
    req.task_description.affordance_info_from.post_transform =
        Eigen::MatrixXd::Map(msg.task_description.affordance_info_from.post_transform.data(), 4, 4); // 4x4 post-multiplication transform
    req.task_description.affordance_info_from.axis_in_final_pose = Eigen::VectorXd::Map(msg.task_description.affordance_info_from.axis_in_final_pose.data(),
                                                                    msg.task_description.affordance_info_from.axis_in_final_pose.size());

    // Convert canonical_pose_from
    req.task_description.canonical_pose_from.method =
        pose_specification_method_from_msg(msg.task_description.canonical_pose_from.method.value);
    req.task_description.canonical_pose_from.frame_name = msg.task_description.canonical_pose_from.frame_name;
    req.task_description.canonical_pose_from.post_transform =
        Eigen::Matrix4d::Map(msg.task_description.canonical_pose_from.post_transform.data(), 4, 4); // 4x4 post-multiplication transform

    // Convert goal
    req.task_description.goal.affordance = msg.task_description.goal.affordance;
    req.task_description.goal.ee_orientation = Eigen::VectorXd::Map(msg.task_description.goal.ee_orientation.data(),
                                                                    msg.task_description.goal.ee_orientation.size());
    req.task_description.goal.canonical_pose =
        Eigen::MatrixXd::Map(msg.task_description.goal.canonical_pose.data(), 4, 4); // 4x4 canonical pose
    req.task_description.goal.gripper = msg.task_description.goal.gripper;

    // Convert other settings
    req.task_description.trajectory_density = msg.task_description.trajectory_density;

    // Convert motion_type and virtual screw order enums
    req.task_description.motion_type = motion_type_from_msg(msg.task_description.motion_type.value);
    req.task_description.vir_screw_order = virtual_screw_order_from_msg(msg.task_description.vir_screw_order.value);

    // Convert gripper_goal_type enum
    req.task_description.gripper_goal_type = gripper_goal_type_from_msg(msg.task_description.gripper_goal_type.value);

    // Convert ee_orientation_constraint enum
    req.task_description.ee_orientation_constraint = ee_orientation_constraint_from_msg(msg.task_description.ee_orientation_constraint.value);

    // Convert KinematicState (robot and gripper)
    req.start_state.robot = Eigen::VectorXd::Map(msg.start_state.robot.data(), msg.start_state.robot.size());
    req.start_state.gripper = msg.start_state.gripper;

    // Copy execute_trajectory (boolean)
    req.execute_trajectory = msg.execute_trajectory;

    return req;
}

// Converts a cca_ros PlanningRequest to a cca_ros_msgs::msg::PlanningRequest
cca_ros_msgs::msg::PlanningRequest convert_req_to_cca_ros_action(const cca_ros::PlanningRequest &req)
{
    cca_ros_msgs::msg::PlanningRequest msg;

    // Convert planning group
    msg.planning_group = req.planning_group;

    // Convert Planner Config
    msg.planner_config.accuracy = req.planner_config.accuracy;
    msg.planner_config.closure_err_threshold_ang = req.planner_config.closure_err_threshold_ang;
    msg.planner_config.closure_err_threshold_lin = req.planner_config.closure_err_threshold_lin;
    msg.planner_config.ik_max_itr = req.planner_config.ik_max_itr;
    msg.planner_config.update_method.value = update_method_to_msg(req.planner_config.update_method);

    // Convert Task Description
    msg.task_description.affordance_info.type.value = screw_type_to_msg(req.task_description.affordance_info.type);

    // Convert axis, location, screw vector to std::vector
    std::copy(req.task_description.affordance_info.axis.data(), req.task_description.affordance_info.axis.data() + 3,
              msg.task_description.affordance_info.axis.begin());

    std::copy(req.task_description.affordance_info.location.data(),
              req.task_description.affordance_info.location.data() + 3,
              msg.task_description.affordance_info.location.begin());

    // You can directly assign the data from the Eigen vector to msg task description screw
    std::copy(req.task_description.affordance_info.screw.data(),
              req.task_description.affordance_info.screw.data() + msg.task_description.affordance_info.screw.size(),
              msg.task_description.affordance_info.screw.begin());

    // Copy other fields
    msg.task_description.affordance_info.pitch = req.task_description.affordance_info.pitch;

   // Convert affordance_info_from
   msg.task_description.affordance_info_from.method.value =
       pose_specification_method_to_msg(req.task_description.affordance_info_from.method);
   msg.task_description.affordance_info_from.frame_name = req.task_description.affordance_info_from.frame_name;
   std::copy(req.task_description.affordance_info_from.post_transform.data(),
             req.task_description.affordance_info_from.post_transform.data() + 16, // 4x4 matrix = 16 elements
             msg.task_description.affordance_info_from.post_transform.begin());
   msg.task_description.affordance_info_from.axis_in_final_pose =
       std::vector<double>(req.task_description.affordance_info_from.axis_in_final_pose.data(),
                          req.task_description.affordance_info_from.axis_in_final_pose.data() + req.task_description.affordance_info_from.axis_in_final_pose.size());
   
   // Convert canonical_pose_from field back to message
   msg.task_description.canonical_pose_from.method.value =
       pose_specification_method_to_msg(req.task_description.canonical_pose_from.method);
   msg.task_description.canonical_pose_from.frame_name = req.task_description.canonical_pose_from.frame_name;
   std::copy(req.task_description.canonical_pose_from.post_transform.data(),
             req.task_description.canonical_pose_from.post_transform.data() + 16, // 4x4 matrix = 16 elements
             msg.task_description.canonical_pose_from.post_transform.begin());

    // Convert goal
    msg.task_description.goal.affordance = req.task_description.goal.affordance;
    std::copy(req.task_description.goal.ee_orientation.data(),
              req.task_description.goal.ee_orientation.data() + req.task_description.goal.ee_orientation.size(),
              msg.task_description.goal.ee_orientation.begin());

    std::copy(req.task_description.goal.canonical_pose.data(),
              req.task_description.goal.canonical_pose.data() + 16, // Assuming 4x4 matrix is flattened to 16 elements
              msg.task_description.goal.canonical_pose.begin());

    msg.task_description.goal.gripper = req.task_description.goal.gripper;
    msg.task_description.trajectory_density = req.task_description.trajectory_density;

    // Convert motion_type and virtual screw order enums
    msg.task_description.motion_type.value = motion_type_to_msg(req.task_description.motion_type);
    msg.task_description.vir_screw_order.value = virtual_screw_order_to_msg(req.task_description.vir_screw_order);

    // Convert gripper_goal_type enum
    msg.task_description.gripper_goal_type.value = gripper_goal_type_to_msg(req.task_description.gripper_goal_type);

    // Convert ee_orientation_constraint enum
    msg.task_description.ee_orientation_constraint.value = ee_orientation_constraint_to_msg(req.task_description.ee_orientation_constraint);

    // Convert KinematicState (robot and gripper)
    msg.start_state.robot =
        std::vector<double>(req.start_state.robot.data(), req.start_state.robot.data() + req.start_state.robot.size());
    msg.start_state.gripper = req.start_state.gripper;

    // Copy execute_trajectory (boolean)
    msg.execute_trajectory = req.execute_trajectory;

    return msg;
}

// Logs a cca ros planning request
std::stringstream log_cca_planning_request(const cca_ros::PlanningRequest &req)
{
    std::stringstream log;
    log << "CcaRos Planning Request:\n";
    log << "-------------------------\n";

    // Planning Group
    log << "Planning Group: " << req.planning_group << "\n";

    // Planner Config
    log << "Planner Config:\n";
    log << "  Accuracy: " << req.planner_config.accuracy << "\n";
    log << "  Closure error threshold (angular): " << req.planner_config.closure_err_threshold_ang << "\n";
    log << "  Closure error threshold (linear): " << req.planner_config.closure_err_threshold_lin << "\n";
    log << "  IK max iterations: " << req.planner_config.ik_max_itr << "\n";
    log << "  Update method: " << update_method_to_string(req.planner_config.update_method) << "\n";

    // Task Description - Affordance Info
    log << "Task Description - Affordance Info:\n";
    log << "  Type: " << screw_type_to_string(req.task_description.affordance_info.type) << "\n";
    log << "  Axis: " << req.task_description.affordance_info.axis.transpose() << "\n";
    log << "  Location: " << req.task_description.affordance_info.location.transpose() << "\n";
    log << "  Screw: " << req.task_description.affordance_info.screw.transpose() << "\n";
    log << "  Pitch: " << req.task_description.affordance_info.pitch << "\n";

    // Affordance Info - From
    log << "  From Method: " << pose_specification_method_to_string(req.task_description.affordance_info_from.method) << "\n";
    log << "  From Frame Name: " << req.task_description.affordance_info_from.frame_name << "\n";
    log << "  From Post Transform:\n" << format_matrix4d(req.task_description.affordance_info_from.post_transform) << "\n";
    log << "  Axis in Final Pose: " << req.task_description.affordance_info_from.axis_in_final_pose.transpose() << "\n";

    // Canonical Pose From
    log << "Canonical Pose From:\n";
    log << "  Method: " << pose_specification_method_to_string(req.task_description.canonical_pose_from.method) << "\n";
    log << "  Frame Name: " << req.task_description.canonical_pose_from.frame_name << "\n";
    log << "  Post Transform:\n" << format_matrix4d(req.task_description.canonical_pose_from.post_transform) << "\n";

    // Task Description - Goal
    log << "Task Description - Goal:\n";
    log << "  Affordance: " << req.task_description.goal.affordance << "\n";
    log << "  EE Orientation: " << req.task_description.goal.ee_orientation.transpose() << "\n";
    log << "  Canonical Pose:\n" << format_matrix4d(req.task_description.goal.canonical_pose) << "\n";
    log << "  Gripper: " << req.task_description.goal.gripper << "\n";

    // Task Description - Other Fields
    log << "  Trajectory Density: " << req.task_description.trajectory_density << "\n";
    log << "  Motion Type: " << motion_type_to_string(req.task_description.motion_type) << "\n";
    log << "  Virtual Screw Order: " << virtual_screw_order_to_string(req.task_description.vir_screw_order) << "\n";
    log << "  Gripper Goal Type: " << gripper_goal_type_to_string(req.task_description.gripper_goal_type) << "\n";
    log << "  Ee Orientation Constraint: " << ee_orientation_constraint_to_string(req.task_description.ee_orientation_constraint) << "\n";

    // Start State
    log << "Start State:\n";
    log << "  Robot: " << req.start_state.robot.transpose() << "\n";
    log << "  Gripper: " << req.start_state.gripper << "\n";

    // Execution Options
    log << "Execution Options:\n";
    log << "  Execute Trajectory: " << std::boolalpha << req.execute_trajectory << "\n";
    log << "-------------------------\n";

    return log;
}

std::stringstream log_cca_planning_result(const cc_affordance_planner::PlannerResult& res)
{
    std::stringstream log;

    const int traj_position_precision = 4; // Number of decimals for the joint trajectory positions
    const int traj_position_field_width = 10; // Minimum number of characters to reserve for position printing

    log << "Planning result:\n";
    
    // Success status
    log << "  Success: " << (res.success ? "true" : "false") << "\n";
    
    // Trajectory description
    log << "  Trajectory description: " << trajectory_description_to_string(res.trajectory_description) << "\n";
    
    // Joint trajectory
    log << "  Closed-Chain joint trajectory:\n";
    log << "    Number of points: " << res.joint_trajectory.size() << "\n";
    log << "    Position trajectory:\n" << std::fixed << std::setprecision(traj_position_precision);
    for (size_t i = 0; i < res.joint_trajectory.size(); ++i) {
        log << "       [" << std::setw(2) << i << "]: ";
        for (int j = 0; j < res.joint_trajectory[i].size(); ++j) {
            log << std::setw(traj_position_field_width) << res.joint_trajectory[i][j];
        }
        log << "\n";
    }
    
    // Whether gripper trajectory is included
    log << "  Includes gripper trajectory: " << (res.includes_gripper_trajectory ? "true" : "false") << "\n";
    
    // Update method
    log << "  Update method: " << update_method_to_string(res.update_method) << "\n";
    
    // Planning time
    log << "  Planning time: " << res.planning_time.count() << " microseconds\n";
    
    // Update trail 
    log << "  Update trail: " << res.update_trail << "\n";

    log << "-------------------------\n";
    
    return log;
}

} // namespace cca_ros_util

namespace
{

cc_affordance_planner::UpdateMethod update_method_from_msg(uint8_t update_method)
{
    switch (update_method)
    {
    case 1:
        return cc_affordance_planner::UpdateMethod::INVERSE;
    case 2:
        return cc_affordance_planner::UpdateMethod::TRANSPOSE;
    case 0:
        return cc_affordance_planner::UpdateMethod::BEST;
    default:
        throw std::invalid_argument("Invalid CCA enum value for conversion.");
    }
}

cc_affordance_planner::MotionType motion_type_from_msg(uint8_t motion_type)
{
    switch (motion_type)
    {
    case 1:
        return cc_affordance_planner::MotionType::APPROACH;
    case 0:
        return cc_affordance_planner::MotionType::AFFORDANCE;
    default:
        throw std::invalid_argument("Invalid CCA enum value for conversion.");
    }
}

affordance_util::PoseSpecificationMethod pose_specification_method_from_msg(uint8_t pose_specification_method)
{
    switch (pose_specification_method)
    {
    case 1:
        return affordance_util::PoseSpecificationMethod::FROM_FK;
    case 2:
        return affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME;
    case 0:
        return affordance_util::PoseSpecificationMethod::PROVIDED;
    default:
        throw std::invalid_argument("Invalid CCA enum value for conversion.");
    }
}
affordance_util::GripperGoalType gripper_goal_type_from_msg(uint8_t gripper_goal_type)
{
    switch (gripper_goal_type)
    {
    case 1:
        return affordance_util::GripperGoalType::CONTINUOUS;
    case 0:
        return affordance_util::GripperGoalType::CONSTANT;
    default:
        throw std::invalid_argument("Invalid AffordanceUtil enum value for conversion.");
    }
}

affordance_util::ScrewType screw_type_from_msg(uint8_t screw_type)
{
    switch (screw_type)
    {
    case 1:
        return affordance_util::ScrewType::ROTATION;
    case 2:
        return affordance_util::ScrewType::TRANSLATION;
    case 3:
        return affordance_util::ScrewType::SCREW;
    case 0:
        return affordance_util::ScrewType::UNSET;
    default:
        throw std::invalid_argument("Invalid AffordanceUtil enum value for conversion.");
    }
}
affordance_util::VirtualScrewOrder virtual_screw_order_from_msg(uint8_t virtual_screw_order)
{
    switch (virtual_screw_order)
    {
    case 1:
        return affordance_util::VirtualScrewOrder::XYZ;
    case 2:
        return affordance_util::VirtualScrewOrder::YZX;
    case 3:
        return affordance_util::VirtualScrewOrder::ZXY;
    case 4:
        return affordance_util::VirtualScrewOrder::XY;
    case 5:
        return affordance_util::VirtualScrewOrder::YZ;
    case 6:
        return affordance_util::VirtualScrewOrder::ZX;
    case 0:
        return affordance_util::VirtualScrewOrder::NONE;
    default:
        throw std::invalid_argument("Invalid AffordanceUtil enum value for conversion.");
    }
}

cc_affordance_planner::EeOrientationConstraint ee_orientation_constraint_from_msg(uint8_t ee_orientation_constraint)
{
    switch (ee_orientation_constraint)
    {
    case 1:
        return cc_affordance_planner::EeOrientationConstraint::PRESERVE;
    case 0:
        return cc_affordance_planner::EeOrientationConstraint::DEFAULT;
    default:
        throw std::invalid_argument("Invalid EeOrientationConstraint enum value for conversion.");
    }
}

uint8_t update_method_to_msg(cc_affordance_planner::UpdateMethod update_method)
{
    switch (update_method)
    {
    case cc_affordance_planner::UpdateMethod::INVERSE:
        return static_cast<uint8_t>(cca_ros_msgs::msg::UpdateMethod::INVERSE);
    case cc_affordance_planner::UpdateMethod::TRANSPOSE:
        return static_cast<uint8_t>(cca_ros_msgs::msg::UpdateMethod::TRANSPOSE);
    case cc_affordance_planner::UpdateMethod::BEST:
        return static_cast<uint8_t>(cca_ros_msgs::msg::UpdateMethod::BEST);
    default:
        throw std::invalid_argument("Invalid CCA UpdateMethod enum value for conversion.");
    }
}

uint8_t motion_type_to_msg(cc_affordance_planner::MotionType motion_type)
{
    switch (motion_type)
    {
    case cc_affordance_planner::MotionType::APPROACH:
        return static_cast<uint8_t>(cca_ros_msgs::msg::MotionType::APPROACH);
    case cc_affordance_planner::MotionType::AFFORDANCE:
        return static_cast<uint8_t>(cca_ros_msgs::msg::MotionType::AFFORDANCE);
    default:
        throw std::invalid_argument("Invalid CCA MotionType enum value for conversion.");
    }
}

uint8_t pose_specification_method_to_msg(affordance_util::PoseSpecificationMethod pose_specification_method)
{
    switch (pose_specification_method)
    {
    case affordance_util::PoseSpecificationMethod::FROM_FK:
        return static_cast<uint8_t>(cca_ros_msgs::msg::PoseSpecificationMethod::FROM_FK);
    case affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME:
        return static_cast<uint8_t>(cca_ros_msgs::msg::PoseSpecificationMethod::FROM_FRAME_NAME);
    case affordance_util::PoseSpecificationMethod::PROVIDED:
        return static_cast<uint8_t>(cca_ros_msgs::msg::PoseSpecificationMethod::PROVIDED);
    default:
        throw std::invalid_argument("Invalid AffordanceUtil PoseSpecificationMethod enum value for conversion.");
    }
}

uint8_t gripper_goal_type_to_msg(affordance_util::GripperGoalType gripper_goal_type)
{
    switch (gripper_goal_type)
    {
    case affordance_util::GripperGoalType::CONTINUOUS:
        return static_cast<uint8_t>(cca_ros_msgs::msg::GripperGoalType::CONTINUOUS);
    case affordance_util::GripperGoalType::CONSTANT:
        return static_cast<uint8_t>(cca_ros_msgs::msg::GripperGoalType::CONSTANT);
    default:
        throw std::invalid_argument("Invalid AffordanceUtil GripperGoalType enum value for conversion.");
    }
}

uint8_t screw_type_to_msg(affordance_util::ScrewType screw_type)
{
    switch (screw_type)
    {
    case affordance_util::ScrewType::ROTATION:
        return static_cast<uint8_t>(cca_ros_msgs::msg::ScrewType::ROTATION);
    case affordance_util::ScrewType::TRANSLATION:
        return static_cast<uint8_t>(cca_ros_msgs::msg::ScrewType::TRANSLATION);
    case affordance_util::ScrewType::SCREW:
        return static_cast<uint8_t>(cca_ros_msgs::msg::ScrewType::SCREW);
    case affordance_util::ScrewType::UNSET:
        return static_cast<uint8_t>(cca_ros_msgs::msg::ScrewType::UNSET);
    default:
        throw std::invalid_argument("Invalid AffordanceUtil ScrewType enum value for conversion.");
    }
}

uint8_t virtual_screw_order_to_msg(affordance_util::VirtualScrewOrder virtual_screw_order)
{
    switch (virtual_screw_order)
    {
    case affordance_util::VirtualScrewOrder::XYZ:
        return static_cast<uint8_t>(cca_ros_msgs::msg::VirtualScrewOrder::XYZ);
    case affordance_util::VirtualScrewOrder::YZX:
        return static_cast<uint8_t>(cca_ros_msgs::msg::VirtualScrewOrder::YZX);
    case affordance_util::VirtualScrewOrder::ZXY:
        return static_cast<uint8_t>(cca_ros_msgs::msg::VirtualScrewOrder::ZXY);
    case affordance_util::VirtualScrewOrder::XY:
        return static_cast<uint8_t>(cca_ros_msgs::msg::VirtualScrewOrder::XY);
    case affordance_util::VirtualScrewOrder::YZ:
        return static_cast<uint8_t>(cca_ros_msgs::msg::VirtualScrewOrder::YZ);
    case affordance_util::VirtualScrewOrder::ZX:
        return static_cast<uint8_t>(cca_ros_msgs::msg::VirtualScrewOrder::ZX);
    case affordance_util::VirtualScrewOrder::NONE:
        return static_cast<uint8_t>(cca_ros_msgs::msg::VirtualScrewOrder::NONE);
    default:
        throw std::invalid_argument("Invalid AffordanceUtil VirtualScrewOrder enum value for conversion.");
    }
}

uint8_t ee_orientation_constraint_to_msg(cc_affordance_planner::EeOrientationConstraint ee_orientation_constraint)
{
    switch (ee_orientation_constraint)
    {
    case cc_affordance_planner::EeOrientationConstraint::PRESERVE:
        return static_cast<uint8_t>(cca_ros_msgs::msg::EeOrientationConstraint::PRESERVE);
    case cc_affordance_planner::EeOrientationConstraint::DEFAULT:
        return static_cast<uint8_t>(cca_ros_msgs::msg::EeOrientationConstraint::DEFAULT);
    default:
        throw std::invalid_argument("Invalid EeOrientationConstraint enum value for conversion.");
    }
}

std::string update_method_to_string(cc_affordance_planner::UpdateMethod method) {
    switch (method)
    {
    case cc_affordance_planner::UpdateMethod::INVERSE:
        return "INVERSE";
    case cc_affordance_planner::UpdateMethod::TRANSPOSE:
        return "TRANSPOSE";
    case cc_affordance_planner::UpdateMethod::BEST:
        return "BEST";
    default:
        return "UNKNOWN";
    }
}

std::string motion_type_to_string(cc_affordance_planner::MotionType type) {
    switch (type)
    {
    case cc_affordance_planner::MotionType::APPROACH:
        return "APPROACH";
    case cc_affordance_planner::MotionType::AFFORDANCE:
        return "AFFORDANCE";
    default:
        return "UNKNOWN";
    }
}

std::string pose_specification_method_to_string(affordance_util::PoseSpecificationMethod method) {
    switch (method)
    {
    case affordance_util::PoseSpecificationMethod::FROM_FK:
        return "FROM_FK";
    case affordance_util::PoseSpecificationMethod::FROM_FRAME_NAME:
        return "FROM_FRAME_NAME";
    case affordance_util::PoseSpecificationMethod::PROVIDED:
        return "PROVIDED";
    default:
        return "UNKNOWN";
    }
}

std::string gripper_goal_type_to_string(affordance_util::GripperGoalType type) {
    switch (type)
    {
    case affordance_util::GripperGoalType::CONTINUOUS:
        return "CONTINUOUS";
    case affordance_util::GripperGoalType::CONSTANT:
        return "CONSTANT";
    default:
        return "UNKNOWN";
    }
}

std::string screw_type_to_string(affordance_util::ScrewType type) {
    switch (type)
    {
    case affordance_util::ScrewType::ROTATION:
        return "ROTATION";
    case affordance_util::ScrewType::TRANSLATION:
        return "TRANSLATION";
    case affordance_util::ScrewType::SCREW:
        return "SCREW";
    case affordance_util::ScrewType::UNSET:
        return "UNSET";
    default:
        return "UNKNOWN";
    }
}

std::string virtual_screw_order_to_string(affordance_util::VirtualScrewOrder order) {
    switch (order)
    {
    case affordance_util::VirtualScrewOrder::XYZ:
        return "XYZ";
    case affordance_util::VirtualScrewOrder::YZX:
        return "YZX";
    case affordance_util::VirtualScrewOrder::ZXY:
        return "ZXY";
    case affordance_util::VirtualScrewOrder::XY:
        return "XY";
    case affordance_util::VirtualScrewOrder::YZ:
        return "YZ";
    case affordance_util::VirtualScrewOrder::ZX:
        return "ZX";
    case affordance_util::VirtualScrewOrder::NONE:
        return "NONE";
    default:
        return "UNKNOWN";
    }
}

std::string ee_orientation_constraint_to_string(cc_affordance_planner::EeOrientationConstraint ee_orientation_constraint) {
    switch (ee_orientation_constraint)
    {
    case cc_affordance_planner::EeOrientationConstraint::DEFAULT:
        return "DEFAULT";
    case cc_affordance_planner::EeOrientationConstraint::PRESERVE:
        return "PRESERVE";
    default:
        return "UNKNOWN";
    }
}

std::string trajectory_description_to_string(cc_affordance_planner::TrajectoryDescription trajectory_description) {
    switch (trajectory_description) {
        case cc_affordance_planner::TrajectoryDescription::FULL:
            return "FULL";
        case cc_affordance_planner::TrajectoryDescription::PARTIAL:
            return "PARTIAL";
        case cc_affordance_planner::TrajectoryDescription::UNSET:
            return "UNSET";
        default:
            return "UNKNOWN";
    }
}

std::string format_matrix4d(const Eigen::Matrix4d& mat, int precision, int width) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision);
    for (int i = 0; i < 4; ++i) {
        ss << "  ";  // Indent each row
        for (int j = 0; j < 4; ++j) {
            ss << std::setw(width) << mat(i, j);
        }
        if (i < 3) ss << "\n";  // No trailing newline on last row
    }
    return ss.str();
};

} // namespace
