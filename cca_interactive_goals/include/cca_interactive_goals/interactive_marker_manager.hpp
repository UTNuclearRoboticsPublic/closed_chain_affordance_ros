#ifndef INTERACTIVE_MARKER_MANAGER_HPP_
#define INTERACTIVE_MARKER_MANAGER_HPP_

// CPP headers
#include <Eigen/Dense>
#include <algorithm>
#include <memory>
#include <string>

// CCA headers
#include <affordance_util/affordance_util.hpp>

// ROS headers
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace interactive_marker_manager
{
enum class ImControlEnable
{
    ROTATION,
    TRANSLATION,
    NONE,
    ALL
};

class InteractiveMarkerManager : public rclcpp::Node
{
  public:
    explicit InteractiveMarkerManager(const std::string &node_name);
    void enable_im_controls(const std::string &marker_name, const ImControlEnable &enable, bool create = false);
    void hide_im(const std::string &marker_name);
    affordance_util::ScrewInfo get_arrow_pose(const std::string &planning_mode, const std::string &axis_mode);
    void draw_ee_or_control_im(const std::string &axis);

  private:
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

    std::string arrow_marker_name_;
    // Some variables and consts to capture arrow pose
    Eigen::Vector3d arrow_axis_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d arrow_location_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    static const Eigen::Vector3d DEFAULT_ARROW_AXIS_;
    static const Eigen::Vector3d DEFAULT_ARROW_LOCATION_;

    // Helper consts
    static const Eigen::Vector3d X_AXIS_;
    static const Eigen::Vector3d Y_AXIS_;
    static const Eigen::Vector3d Z_AXIS_;
    static const Eigen::Vector3d NEG_X_AXIS_;
    static const Eigen::Vector3d NEG_Y_AXIS_;
    static const Eigen::Vector3d NEG_Z_AXIS_;
    static const std::map<std::string, Eigen::Quaterniond>
        AXIS_ORIENTATION_MAP; ///<--map containing orientation transform to align x-axis with various axes

    // ARROW AESTHETICS -- Color is Cyan
    static constexpr double ARROW_SCALE = 0.5;
    static constexpr double ARROW_COLOR_R = 0.251;
    static constexpr double ARROW_COLOR_G = 0.878;
    static constexpr double ARROW_COLOR_B = 0.816;

    // Methods
    void process_arrow_feedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);
};

} // namespace interactive_marker_manager

#endif
