#ifndef CCA_PANEL_HPP
#define CCA_PANEL_HPP

#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <plan_req_builder/plan_req_builder.hpp>
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util/affordance_util.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cc_affordance_planner/cc_affordance_planner_interface.hpp>
#include <cca_ros/cca_ros.hpp>
#include <chrono>
#include <tf2_eigen/tf2_eigen.hpp>

#include "interactive_goal_interfaces/msg/button_press.hpp"
#include "interactive_goal_interfaces/msg/screw_info.hpp"
#include "interactive_goal_interfaces/msg/advanced_settings.hpp"

class CcaPanel : public cca_ros::CcaRos
{
public:
  explicit CcaPanel(const std::string& node_name, const rclcpp::NodeOptions& node_options);

private:
  rclcpp::Subscription<interactive_goal_interfaces::msg::ButtonPress>::SharedPtr button_press_subscriber_;
  rclcpp::Subscription<interactive_goal_interfaces::msg::ScrewInfo>::SharedPtr screw_info_subscriber_;
  rclcpp::Subscription<interactive_goal_interfaces::msg::AdvancedSettings>::SharedPtr settings_subscriber_;
  rclcpp::Subscription<visualization_msgs::msg::InteractiveMarkerFeedback>::SharedPtr
      interactive_marker_feedback_subscriber_;
  rclcpp::Publisher<interactive_goal_interfaces::msg::ButtonPress>::SharedPtr process_success_publisher_;

  interactive_goal_interfaces::msg::ScrewInfo current_screw_info_;
  interactive_goal_interfaces::msg::AdvancedSettings current_settings_;
  geometry_msgs::msg::Pose current_arrow_pose_;
  geometry_msgs::msg::Pose current_frame_pose_;
  cc_affordance_planner::PlannerConfig config;
  int current_trajectory_density;
  affordance_util::VirtualScrewOrder current_screw_order;

  void buttonPressCallback(const interactive_goal_interfaces::msg::ButtonPress::SharedPtr msg);
  void screwInfoCallback(const interactive_goal_interfaces::msg::ScrewInfo::SharedPtr msg);
  void settingsCallback(const interactive_goal_interfaces::msg::AdvancedSettings::SharedPtr msg);
  void interactiveMarkerFeedbackCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback);
  void block_until_trajectory_execution();
  bool run(const cca_ros::PlanningRequest& planning_request);
  cca_ros::PlanningRequest preparePlanningRequest(const bool visualize, const bool execute);
};

#endif  // CCA_PANEL_HPP
