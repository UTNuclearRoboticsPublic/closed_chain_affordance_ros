#include "cca_ros_viz/interactive_marker_manager.hpp"

namespace interactive_marker_manager
{

// Initialize Eigen static consts
const Eigen::Vector3d InteractiveMarkerManager::X_AXIS_(1.0, 0.0, 0.0);
const Eigen::Vector3d InteractiveMarkerManager::Y_AXIS_(0.0, 1.0, 0.0);
const Eigen::Vector3d InteractiveMarkerManager::Z_AXIS_(0.0, 0.0, 1.0);
const Eigen::Vector3d InteractiveMarkerManager::NEG_X_AXIS_(-1.0, 0.0, 0.0);
const Eigen::Vector3d InteractiveMarkerManager::NEG_Y_AXIS_(0.0, -1.0, 0.0);
const Eigen::Vector3d InteractiveMarkerManager::NEG_Z_AXIS_(0.0, 0.0, -1.0);
const Eigen::Vector3d InteractiveMarkerManager::DEFAULT_ARROW_AXIS_ = InteractiveMarkerManager::X_AXIS_;
const Eigen::Vector3d InteractiveMarkerManager::DEFAULT_ARROW_LOCATION_(0.0, 0.0, 0.0);
const std::map<std::string, Eigen::Quaterniond> InteractiveMarkerManager::AXIS_ORIENTATION_MAP = {
    {"x", Eigen::Quaterniond::FromTwoVectors(X_AXIS_, X_AXIS_)},      // No rotation needed
    {"y", Eigen::Quaterniond::FromTwoVectors(X_AXIS_, Y_AXIS_)},      // Rotate X to Y
    {"z", Eigen::Quaterniond::FromTwoVectors(X_AXIS_, Z_AXIS_)},      // Rotate X to Z
    {"-x", Eigen::Quaterniond::FromTwoVectors(X_AXIS_, NEG_X_AXIS_)}, // Rotate X to -X
    {"-y", Eigen::Quaterniond::FromTwoVectors(X_AXIS_, NEG_Y_AXIS_)}, // Rotate X to -Y
    {"-z", Eigen::Quaterniond::FromTwoVectors(X_AXIS_, NEG_Z_AXIS_)}  // Rotate X to -Z
};

InteractiveMarkerManager::InteractiveMarkerManager(const std::string &node_name) : rclcpp::Node(node_name)
{

    // Initialize servers and clients
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "interactive_goals", this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Enable the arrow
    ImControlEnableInfo arrow_enable_info;
    arrow_enable_info.marker_name = arrow_marker_name_;
    arrow_enable_info.enable = ImControlEnable::ALL;
    arrow_enable_info.create = true;
    enable_im_controls(arrow_enable_info);

    try
    {
        this->declare_parameter<std::string>("tool_frame");
        tool_frame_name_ = this->get_parameter("tool_frame").as_string();
        this->declare_parameter<std::string>("ref_frame");
        ref_frame_name_ = this->get_parameter("ref_frame").as_string();
        this->declare_parameter<std::string>("ee_frame");
        ee_frame_name_ = this->get_parameter("ee_frame").as_string();
        this->declare_parameter<std::vector<double>>("ee_to_tool_offset");
        auto tool_offset = this->get_parameter("ee_to_tool_offset").as_double_array();
        ee_to_tool_offset_ =
            Eigen::Vector3d(tool_offset[0], tool_offset[1], tool_offset[2]); // Location of tool in the EE frame
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to get parameter 'tool_frame' or 'ref_frame': %s. You may have forgetten to load the "
                     "cca_<robot>_ros_viz_setup.yaml file with the "
                     "rviz2 node.",
                     e.what());
    }

    // Initialize the tf broadcaster and timer to publish transform between the EE and tool frames
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(tf_publish_rate_),
                                     std::bind(&InteractiveMarkerManager::publish_transform_, this));
    RCLCPP_INFO(this->get_logger(), "Interactive marker manager initialized.");
}

void InteractiveMarkerManager::process_arrow_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
    switch (feedback->event_type)
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:

        // Get orientation transform (this is wrt default x-axis)
        Eigen::Quaterniond q(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                             feedback->pose.orientation.z);
        arrow_axis_ = q * X_AXIS_;
        arrow_location_ =
            Eigen::Vector3d(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

        server_->setPose(feedback->marker_name, feedback->pose);
        server_->applyChanges();
        break;
    }
}

void InteractiveMarkerManager::enable_im_controls(const ImControlEnableInfo &info)
{
    visualization_msgs::msg::InteractiveMarker int_marker;

    // Retrieve marker if not create a new one
    if (!info.create)
    {
        server_->get(info.marker_name, int_marker);
    }

    // Clear and initialize marker
    if (info.reset)
    {
        int_marker = visualization_msgs::msg::InteractiveMarker();

        // Reset recorded arrow pose as well
        arrow_axis_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        arrow_location_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    }

    int_marker.header.frame_id = ref_frame_name_;
    int_marker.name = info.marker_name;
    int_marker.description = "";
    int_marker.scale = ARROW_SCALE;

    // Draw in tool frame if asked
    if (info.in_tool_frame)
    {
        const Eigen::Isometry3d aff_htm = ros_cpp_util::get_htm(ref_frame_name_, tool_frame_name_, *tf_buffer_);
        int_marker.pose.position.x = aff_htm.translation().x();
        int_marker.pose.position.y = aff_htm.translation().y();
        int_marker.pose.position.z = aff_htm.translation().z();
        if (aff_htm.matrix().isApprox(Eigen::Matrix4d::Identity()))
        {
            RCLCPP_ERROR(this->get_logger(), "Could not lookup %s frame. Will place arrow at %s instead.",
                         tool_frame_name_.c_str(), ref_frame_name_.c_str());
        }
    }

    // Lambda to add control using static axis vectors
    auto addControl = [&](const std::string &name, const Eigen::Vector3d &axis, bool isRotation) {
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.orientation.w = 1.0;
        control.orientation.x = axis.x();
        control.orientation.y = axis.y();
        control.orientation.z = axis.z();
        control.name = name;
        control.interaction_mode = isRotation ? visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS
                                              : visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    };

    // Create and add arrow marker
    visualization_msgs::msg::Marker arrow;
    arrow.ns = "interactive_goals";
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = ARROW_SCALE;
    arrow.scale.y = ARROW_SCALE / 10.0;
    arrow.scale.z = ARROW_SCALE / 10.0;
    arrow.color.r = ARROW_COLOR_R;
    arrow.color.g = ARROW_COLOR_G;
    arrow.color.b = ARROW_COLOR_B;
    arrow.color.a = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl arrow_control;
    arrow_control.always_visible = true;
    arrow_control.markers.push_back(arrow);
    int_marker.controls.push_back(arrow_control);

    // Helper to add rotation and translation controls
    auto addRotationControls = [&]() {
        addControl("rotate_x", X_AXIS_, true);
        addControl("rotate_y", Y_AXIS_, true);
        addControl("rotate_z", Z_AXIS_, true);
    };

    auto addTranslationControls = [&]() {
        addControl("move_x", X_AXIS_, false);
        addControl("move_y", Y_AXIS_, false);
        addControl("move_z", Z_AXIS_, false);
    };

    // Configure controls based on enable type
    switch (info.enable)
    {
    case ImControlEnable::ROTATION:
        addRotationControls();
        break;
    case ImControlEnable::TRANSLATION:
        addTranslationControls();
        break;
    case ImControlEnable::ALL:
        addRotationControls();
        addTranslationControls();
        break;
    case ImControlEnable::NONE:
    default:
        break;
    }

    // Insert and apply changes
    if (info.create)
    { // Insert with callback if it does not exist
        server_->insert(int_marker,
                        std::bind(&InteractiveMarkerManager::process_arrow_feedback, this, std::placeholders::_1));
    }
    else
    {
        server_->insert(int_marker);
    }
    server_->applyChanges();
}

void InteractiveMarkerManager::hide_im(const std::string &marker_name)
{
    // Disable interactive marker controls
    ImControlEnableInfo arrow_enable_info;
    arrow_enable_info.marker_name = marker_name;
    arrow_enable_info.enable = ImControlEnable::NONE;
    enable_im_controls(arrow_enable_info);

    // Get the interactive marker object
    visualization_msgs::msg::InteractiveMarker int_marker;
    server_->get(marker_name, int_marker);

    // Hide all non-interactive makers by making them fully transparent
    std::for_each(int_marker.controls.begin(), int_marker.controls.end(), [](auto &control) {
        std::for_each(control.markers.begin(), control.markers.end(), [](auto &marker) { marker.color.a = 0.0; });
    });

    server_->insert(int_marker);
    server_->applyChanges();
}

void InteractiveMarkerManager::draw_ee_or_control_im(const std::string &axis)
{
    ImControlEnableInfo arrow_enable_info;
    arrow_enable_info.marker_name = arrow_marker_name_;
    arrow_enable_info.in_tool_frame = true;
    // Enable interactive marker for manual mode (with rotation control) and return
    if (axis == "Interactive Axis")
    {
        arrow_enable_info.enable = ImControlEnable::ROTATION;
        enable_im_controls(arrow_enable_info);
        return;
    }

    // Enable the arrow with no interactive control
    arrow_enable_info.enable = ImControlEnable::NONE;
    enable_im_controls(arrow_enable_info);

    // Get the interactive marker
    visualization_msgs::msg::InteractiveMarker int_marker;
    server_->get(arrow_marker_name_, int_marker);

    auto &marker = int_marker.controls.front().markers.front(); // First control contains the arrow marker

    // Orient the arrow to align with the specified axis
    if (AXIS_ORIENTATION_MAP.find(axis) != AXIS_ORIENTATION_MAP.end())
    {
        Eigen::Quaterniond q = InteractiveMarkerManager::AXIS_ORIENTATION_MAP.at(axis);
        marker.pose.orientation.w = q.w();
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
    }

    // Update the interactive marker with the new arrow orientation
    server_->insert(int_marker);
    server_->applyChanges();
}

affordance_util::ScrewInfo InteractiveMarkerManager::get_arrow_pose(const std::string &planning_mode,
                                                                    const std::string &axis_mode)
{
    affordance_util::ScrewInfo screw_info;

    // Check if asked to look at the interactive marker
    if ((planning_mode == "EE Orientation Only") && (axis_mode != "Interactive Axis"))
    {

        // Retrieve the marker
        visualization_msgs::msg::InteractiveMarker int_marker;
        server_->get(arrow_marker_name_, int_marker);
        auto &marker = int_marker.controls.front().markers.front(); // First control contains the arrow marker

        // Extract the marker pose
        const geometry_msgs::msg::Pose &marker_pose = marker.pose;
        Eigen::Vector3d arrow_location(marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
        Eigen::Quaterniond arrow_quaternion(marker_pose.orientation.w, marker_pose.orientation.x,
                                            marker_pose.orientation.y, marker_pose.orientation.z);

        // Determine how the quaternion has transformed the arrow
        Eigen::Vector3d arrow_axis = arrow_quaternion * DEFAULT_ARROW_AXIS_;

        // Populate the screw_info struct
        screw_info.axis = arrow_axis;
        screw_info.location = arrow_location;
    }
    else // from interactive marker
    {
        // Go with default location if the arrow hasn't moved
        if (arrow_axis_.hasNaN() && arrow_location_.hasNaN())
        {
            screw_info.axis = DEFAULT_ARROW_AXIS_;
            screw_info.location = DEFAULT_ARROW_LOCATION_;
        }
        else
        {
            screw_info.axis = arrow_axis_;
            screw_info.location = arrow_location_;
        }
    }
    return screw_info;
}
void InteractiveMarkerManager::publish_transform_()
{
    // Create the transform message
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set header details
    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = ee_frame_name_;
    transform_stamped.child_frame_id = tool_frame_name_;

    // Set translation (x, y, z)
    transform_stamped.transform.translation.x = ee_to_tool_offset_[0];
    transform_stamped.transform.translation.y = ee_to_tool_offset_[1];
    transform_stamped.transform.translation.z = ee_to_tool_offset_[2];

    // We assume orientation same as ee_frame_
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    // Publish the transform
    tf_broadcaster_->sendTransform(transform_stamped);
}
} // namespace interactive_marker_manager
