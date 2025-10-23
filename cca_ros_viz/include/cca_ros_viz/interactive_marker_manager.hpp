///////////////////////////////////////////////////////////////////////////////
//      Title     : interactive_marker_manager.hpp
//      Project   : interactive_marker_manager
//      Created   : Spring 2025
//      Author    : Janak Panthi (Crasun Jans)
///////////////////////////////////////////////////////////////////////////////

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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace interactive_marker_manager
{
/**
 * @brief Enum representing options to enable different parts of an interactive marker.
 */
enum class ImControlEnable
{
    ROTATION,    ///< Enables rotation control
    TRANSLATION, ///< Enables translation control
    NONE,        ///< Disables all interactive controls
    ALL          ///< Enables both rotation and translation controls
};

/**
 * @brief Struct that holds information for enabling an interactive marker.
 */
struct ImControlEnableInfo
{
    std::string marker_name;    ///< Name of the interactive marker
    ImControlEnable enable;     ///< What aspects of the interactive marker to enable
    bool create = false;        ///< Whether to create the marker
    bool reset = true;          ///< Whether to reset the marker pose
    bool in_tool_frame = false; ///< Whether to draw in the tool frame
};

class InteractiveMarkerManager : public rclcpp::Node
{
  public:
    explicit InteractiveMarkerManager(const std::string &node_name);

    /**
     * @brief Enables or creates an interactive marker based on the provided parameters.
     *
     * @param info A struct containing information on how to enable the interactive marker.
     *             This includes the marker's name, the parts to enable, and options for creation,
     *             resetting the pose, and drawing in the tool frame.
     *
     * @note The `info` struct holds the following fields:
     *   - `marker_name`: The name of the interactive marker.
     *   - `enable`: Specifies which parts of the interactive marker to enable.
     *   - `create`: A boolean indicating whether to create the marker (default is `false`).
     *   - `reset`: A boolean specifying whether to reset the marker's pose (default is `true`).
     *   - `in_tool_frame`: A boolean indicating whether to draw the marker in the tool frame (default is `false`).
     */
    void enable_im_controls(const ImControlEnableInfo &info);

    /**
     * @brief Hides the specified interactive marker.
     *
     * @param marker_name The name of the marker to hide.
     */
    void hide_im(const std::string &marker_name);

    /**
     * @brief Returns the pose of the arrow representing the screw axis based on the given CCA planning and axis modes.
     *
     * @param planning_mode The planning mode from the CCA planning plugin.
     * @param axis_mode The axis mode from the CCA planning plugin EE Orientation Control Axis option.
     *
     * @return The screw axis information, including its axis and location.
     */
    affordance_util::ScrewInfo get_arrow_pose(const std::string &planning_mode, const std::string &axis_mode);

    /**
    * @brief Returns the current pose of the affordance start frame for CCA APPROACH-type planning.
    *
    * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix representing the affordance start frame in the CCA (robot) reference frame.
    */
    Eigen::Matrix4d get_frame_pose();

    /**
     * @brief Draws the interactive marker for the CCA planning plugin EE Orientation Control mode based on the given
     * axis name.
     *
     * @param axis An axis option from the CCA planning plugin EE Orientation Control mode.
     */
    void draw_ee_or_control_im(const std::string &axis);

  protected:
    static constexpr const char *marker_namespace_ =
        "interactive_goals"; ///< Namespace for the markers
    static constexpr const char *arrow_marker_name_ =
        "affordance_screw"; ///< Name of the interactive marker for the screw arrow
    static constexpr const char *frame_marker_name_ =
        "affordance_start_frame"; ///< Name of the interactive marker for the screw start frame

  private:
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_; ///< Server managing interactive markers
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;                           ///< TF2 buffer for transformation lookup
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};     ///< TF2 transform listener
    rclcpp::TimerBase::SharedPtr timer_;                                   ///< Timer to publish transform at a set rate
    const std::chrono::milliseconds tf_publish_rate_{100};                 ///< TF publish rate
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;        ///< Transform broadcaster

    std::string tool_frame_name_;       ///< This is where the arrow will appear in "EE Orientation Only" planning mode
    std::string ref_frame_name_;        ///< This is where the arrow will appear first in the "Affordance" planning mode
    std::string ee_frame_name_;         ///< Name of the EE frame
    Eigen::Vector3d ee_to_tool_offset_; ///< Location of the tool in the EE frame

    // Variables for capturing the arrow pose
    Eigen::Vector3d arrow_axis_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d arrow_location_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    static const Eigen::Vector3d DEFAULT_ARROW_AXIS_;
    static const Eigen::Vector3d DEFAULT_ARROW_LOCATION_;

    // Helper constants
    static const Eigen::Vector3d X_AXIS_;
    static const Eigen::Vector3d Y_AXIS_;
    static const Eigen::Vector3d Z_AXIS_;
    static const Eigen::Vector3d NEG_X_AXIS_;
    static const Eigen::Vector3d NEG_Y_AXIS_;
    static const Eigen::Vector3d NEG_Z_AXIS_;
    static const std::map<std::string, Eigen::Quaterniond>
        AXIS_ORIENTATION_MAP; ///< Map containing orientation transformations to align the x-axis with various axes

    // Arrow aesthetics -- Color is Cyan
    static constexpr double ARROW_SCALE_ = 0.5;
    static constexpr double ARROW_COLOR_R_ = 0.251;
    static constexpr double ARROW_COLOR_G_ = 0.878;
    static constexpr double ARROW_COLOR_B_ = 0.816;

    // Variables for capturing the frame pose
    Eigen::Matrix4d frame_pose_ = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
    static const Eigen::Matrix4d DEFAULT_FRAME_POSE_;

    // Frame geometry
    static constexpr double FRAME_SCALE_ = 0.2;
    static constexpr double ARROW_TO_FRAME_OFFSET_X_ = 0.1;
    static constexpr double ARROW_TO_FRAME_OFFSET_Y_ = 0.1;
    static constexpr double ARROW_TO_FRAME_OFFSET_Z_ = 0.1;

    /**
     * @brief Processes feedback from the arrow interactive marker.
     *
     * @param feedback The feedback from the interactive marker.
     */
    void process_arrow_feedback_(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

    /**
     * @brief Processes feedback from the frame interactive marker.
     *
     * @param feedback The feedback from the interactive marker.
     */
    void process_frame_feedback_(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

    /**
     * @brief Publishes transform between the EE and tool frame
     */
    void publish_transform_();
};

} // namespace interactive_marker_manager

#endif
