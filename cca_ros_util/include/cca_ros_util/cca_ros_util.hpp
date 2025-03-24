///////////////////////////////////////////////////////////////////////////////
//      Title     : cca_ros_util.hpp
//      Project   : cca_ros_util
//      Created   : Spring 2025
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

#ifndef CCA_ROS_UTIL_H
#define CCA_ROS_UTIL_H

#include "affordance_util/affordance_util.hpp"
#include "cca_ros/cca_ros.hpp"
#include "cca_ros_msgs/msg/gripper_goal_type.hpp"
#include "cca_ros_msgs/msg/motion_type.hpp"
#include "cca_ros_msgs/msg/planning_request.hpp"
#include "cca_ros_msgs/msg/update_method.hpp"
#include "cca_ros_msgs/msg/virtual_screw_order.hpp"
#include <Eigen/Dense>
#include <stdexcept>
#include <unordered_map>

namespace cca_ros_util
{

/**
 * @brief Converts a ROS PlanningRequest message to a CCA PlanningRequest object.
 *
 * This function takes a cca_ros_msgs::msg::PlanningRequest message and converts it
 * into a cca_ros::PlanningRequest struct for use with CCA libraries.
 *
 * @param msg The input PlanningRequest message containing planning data.
 * @return A CCA PlanningRequest object for further processing.
 */
cca_ros::PlanningRequest convert_cca_ros_action_to_req(const cca_ros_msgs::msg::PlanningRequest &msg);

/**
 * @brief Converts a CCA PlanningRequest object to a ROS PlanningRequest message.
 *
 * This function transforms a cca_ros::PlanningRequest object into a
 * cca_ros_msgs::msg::PlanningRequest message for communication within ROS.
 *
 * @param req The CCA PlanningRequest object to be converted.
 * @return A ROS PlanningRequest message.
 */
cca_ros_msgs::msg::PlanningRequest convert_req_to_cca_ros_action(const cca_ros::PlanningRequest &req);

/**
 * @brief Logs the details of a CCA PlanningRequest object as a formatted string.
 *
 * Generates a detailed, human-readable representation of the provided
 * cca_ros::PlanningRequest object for debugging and logging purposes.
 *
 * @param req The CCA PlanningRequest object to log.
 * @return A std::stringstream containing the formatted log output.
 */
std::stringstream log_cca_planning_request(const cca_ros::PlanningRequest &req);

} // namespace cca_ros_util

namespace
{
// Helper functions restricted to this file
cc_affordance_planner::UpdateMethod update_method_from_msg(uint8_t update_method);
cc_affordance_planner::MotionType motion_type_from_msg(uint8_t motion_type);
affordance_util::ScrewLocationMethod screw_location_method_from_msg(uint8_t location_method);
affordance_util::GripperGoalType gripper_goal_type_from_msg(uint8_t gripper_goal_type);
affordance_util::ScrewType screw_type_from_msg(uint8_t screw_type);
affordance_util::VirtualScrewOrder virtual_screw_order_from_msg(uint8_t virtual_screw_order);
uint8_t update_method_to_msg(cc_affordance_planner::UpdateMethod update_method);
uint8_t motion_type_to_msg(cc_affordance_planner::MotionType motion_type);
uint8_t screw_location_method_to_msg(affordance_util::ScrewLocationMethod location_method);
uint8_t gripper_goal_type_to_msg(affordance_util::GripperGoalType gripper_goal_type);
uint8_t screw_type_to_msg(affordance_util::ScrewType screw_type);
uint8_t virtual_screw_order_to_msg(affordance_util::VirtualScrewOrder virtual_screw_order);

} // namespace

#endif // CCA_ROS_UTIL_H
