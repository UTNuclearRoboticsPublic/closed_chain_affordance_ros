#ifndef CCA_ROS_UTIL_H
#define CCA_ROS_UTIL_H

#include <unordered_map>
#include <stdexcept>
#include <Eigen/Dense>
#include "cca_ros/cca_ros.hpp" 
#include "cca_ros_viz_msgs/msg/update_method.hpp"
#include "cca_ros_viz_msgs/msg/motion_type.hpp"
#include "cca_ros_viz_msgs/msg/virtual_screw_order.hpp"
#include "cca_ros_viz_msgs/msg/gripper_goal_type.hpp"
#include "cca_ros_viz_msgs/msg/planning_request.hpp"
#include "affordance_util/affordance_util.hpp" 

namespace cca_ros_util {

// Converts a ROS action to a cca_ros PlanningRequest
cca_ros::PlanningRequest convert_cca_ros_action_to_req(const cca_ros_viz_msgs::msg::PlanningRequest& msg);

cc_affordance_planner::UpdateMethod convert_ros_update_method_enum_to_cca(int update_method);

cc_affordance_planner::MotionType convert_ros_motion_type_enum_to_cca(int motion_type);

affordance_util::ScrewLocationMethod convert_ros_screw_location_method_enum_to_affordance_util(int location_method);

affordance_util::GripperGoalType convert_ros_gripper_goal_type_enum_to_affordance_util(int gripper_goal_type);

affordance_util::ScrewType convert_ros_screw_type_enum_to_affordance_util(int screw_type);

affordance_util::VirtualScrewOrder convert_ros_virtual_screw_order_enum_to_affordance_util(int virtual_screw_order);

// Function to convert cca_ros::PlanningRequest to CcaRosAction
cca_ros_viz_msgs::msg::PlanningRequest convert_req_to_cca_ros_action(const cca_ros::PlanningRequest& req);

// Template function for converting CCA enums to ROS enums
uint8_t convert_cca_update_method_to_ros(cc_affordance_planner::UpdateMethod update_method);
uint8_t convert_cca_motion_type_to_ros(cc_affordance_planner::MotionType motion_type);
uint8_t convert_affordance_util_screw_location_method_to_ros(affordance_util::ScrewLocationMethod location_method);
uint8_t convert_affordance_util_gripper_goal_type_to_ros(affordance_util::GripperGoalType gripper_goal_type);
uint8_t convert_affordance_util_screw_type_to_ros(affordance_util::ScrewType screw_type);
uint8_t convert_affordance_util_virtual_screw_order_to_ros(affordance_util::VirtualScrewOrder virtual_screw_order);

} // namespace cca_ros_util

#endif // CCA_ROS_UTIL_H

