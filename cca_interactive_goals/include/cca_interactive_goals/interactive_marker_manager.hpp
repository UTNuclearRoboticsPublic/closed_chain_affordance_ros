#ifndef INTERACTIVE_MARKER_MANAGER_HPP_
#define INTERACTIVE_MARKER_MANAGER_HPP_

//CPP headers
#include <memory>
#include <string>
#include <Eigen/Dense>
#include <algorithm>

//CCA headers
#include <affordance_util/affordance_util.hpp>

//ROS headers
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace interactive_marker_manager
{
  enum class ImControlEnable{ROTATION, TRANSLATION, NONE, ALL};

class InteractiveMarkerManager : public rclcpp::Node 
{
public:

  explicit InteractiveMarkerManager(const std::string& node_name);
  void enable_im_controls(const std::string& marker_name, const ImControlEnable& enable, bool create = false);
  void hide_im(const std::string& marker_name);
  affordance_util::ScrewInfo get_arrow_pose(const std::string planning_mode, const std::string axis_mode);
  void draw_ee_or_control_im(int index);

private:
  const int marker_id_=8;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  // Some variables and consts to capture arrow pose
  Eigen::Vector3d arrow_axis_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d arrow_location_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  static const Eigen::Vector3d DEFAULT_ARROW_AXIS_;
  static const Eigen::Vector3d DEFAULT_ARROW_LOCATION_;

  // Helper consts
  static const Eigen::Vector3d X_AXIS_;

  // Methods
  void process_arrow_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
};


}  // namespace interactive_marker_manager

#endif  
