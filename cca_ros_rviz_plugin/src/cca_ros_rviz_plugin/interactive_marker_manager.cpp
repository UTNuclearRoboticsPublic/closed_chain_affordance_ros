#include "cca_ros_rviz_plugin/interactive_marker_manager.hpp"

namespace interactive_marker_manager
{

// Initialize Eigen static consts
const Eigen::Vector3d InteractiveMarkerManager::X_AXIS_ = affordance_util::axis_to_vec(affordance_util::Axis::X);
const Eigen::Vector3d InteractiveMarkerManager::Y_AXIS_ = affordance_util::axis_to_vec(affordance_util::Axis::Y);
const Eigen::Vector3d InteractiveMarkerManager::Z_AXIS_ = affordance_util::axis_to_vec(affordance_util::Axis::Z);
const Eigen::Vector3d InteractiveMarkerManager::NEG_X_AXIS_= affordance_util::axis_to_vec(affordance_util::Axis::X_MINUS);
const Eigen::Vector3d InteractiveMarkerManager::NEG_Y_AXIS_= affordance_util::axis_to_vec(affordance_util::Axis::Y_MINUS);
const Eigen::Vector3d InteractiveMarkerManager::NEG_Z_AXIS_= affordance_util::axis_to_vec(affordance_util::Axis::Z_MINUS);
const Eigen::Vector3d InteractiveMarkerManager::DEFAULT_ARROW_AXIS_ = InteractiveMarkerManager::X_AXIS_;
const Eigen::Vector3d InteractiveMarkerManager::DEFAULT_ARROW_LOCATION_(0.0, 0.0, 0.0);
const Eigen::Matrix4d InteractiveMarkerManager::DEFAULT_FRAME_POSE_ =
    (Eigen::Matrix4d() <<
        1.0, 0.0, 0.0, ARROW_TO_FRAME_OFFSET_X_,
        0.0, 1.0, 0.0, ARROW_TO_FRAME_OFFSET_Y_,
        0.0, 0.0, 1.0, ARROW_TO_FRAME_OFFSET_Z_,
        0.0, 0.0, 0.0, 1.0
    ).finished();
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

    // Initialize the tf broadcaster so we could publish static transforms
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Extract frame names for planning groups and publish static transforms between EE and tool frames and for each planning group
    using namespace ros_cpp_util;
    cca_planning_groups_ = get_required_str_array_param(this, "cca_planning_groups");
    const std::string pg_prefix = "cca_planning_group_info.";

    for (const auto& pg_name : cca_planning_groups_) {

        // Extract frame names for this planning group
        const std::string param_prefix = pg_prefix + pg_name;

        PlanningGroupFrameInfo pg_frame_info;
        pg_frame_info.ref_frame = get_required_str_param(this, param_prefix + ".ref_frame");
        pg_frame_info.tool_frame = get_required_str_param(this, param_prefix + ".tool.frame");

        // Add to map
        planning_group_frame_info_map_[pg_name] = pg_frame_info;

        // Publish static transform between EE and tool frame
        const std::string& ee_frame = get_required_str_param(this, param_prefix + ".end_effector.frame");
        const Eigen::Vector3d& ee_to_tool_offset = Eigen::Vector3d(get_required_double_array_param(this, param_prefix + ".tool.offset_from_ee_frame").data());
        if (ee_frame!=pg_frame_info.tool_frame){ // Avoid publishing if both frames are the same
            this->publish_transform_(ee_frame, pg_frame_info.tool_frame, ee_to_tool_offset);
	}
    }

    default_planning_group_ = cca_planning_groups_.front(); // We will use the first planning group as default

    // Enable the arrow
    ImControlEnableInfo arrow_enable_info;
    arrow_enable_info.marker_name = arrow_marker_name_;
    arrow_enable_info.enable = ImControlEnable::ALL;
    arrow_enable_info.create = true;
    enable_im_controls(arrow_enable_info, default_planning_group_);

    // Enable the frame
    ImControlEnableInfo frame_enable_info;
    frame_enable_info.marker_name = frame_marker_name_;
    frame_enable_info.enable = ImControlEnable::ALL;
    frame_enable_info.create = true;
    frame_enable_info.in_tool_frame = true;
    enable_im_controls(frame_enable_info, default_planning_group_);

    RCLCPP_INFO(this->get_logger(), "Interactive marker manager initialized.");
}

void InteractiveMarkerManager::process_arrow_feedback_(
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

void InteractiveMarkerManager::process_frame_feedback_(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
{
    switch (feedback->event_type)
    {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:

        // Get orientation
        Eigen::Quaterniond q(feedback->pose.orientation.w,
                             feedback->pose.orientation.x,
                             feedback->pose.orientation.y,
                             feedback->pose.orientation.z);

        // Get position
	Eigen::Vector3d p(feedback->pose.position.x,
		          feedback->pose.position.y,
		          feedback->pose.position.z);

        // Update the frame pose
	frame_pose_.setIdentity();
        frame_pose_.block<3, 3>(0, 0) = q.toRotationMatrix();
        frame_pose_.block<3, 1>(0, 3) = p;

        // Update marker pose in server
        server_->setPose(feedback->marker_name, feedback->pose);
        server_->applyChanges();
        break;

    }
}


void InteractiveMarkerManager::enable_im_controls(const ImControlEnableInfo &info, const std::string& planning_group)
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

        // Reset recorded pose as well
        if (info.marker_name==arrow_marker_name_){

            arrow_axis_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            arrow_location_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());

	} else if (info.marker_name==frame_marker_name_){

	    frame_pose_ = Eigen::Matrix4d::Constant(std::numeric_limits<double>::quiet_NaN());
	}
    }

    const std::string& ref_frame_name = planning_group_frame_info_map_.at(planning_group).ref_frame;
    const std::string& tool_frame_name = planning_group_frame_info_map_.at(planning_group).tool_frame;
    int_marker.header.frame_id = info.in_tool_frame ? tool_frame_name : ref_frame_name;
    int_marker.header.stamp = this->now();

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

    // Declare control and make it always visible
    visualization_msgs::msg::InteractiveMarkerControl im_control;
    im_control.always_visible = true;

    if (info.marker_name==arrow_marker_name_){

    	int_marker.name = arrow_marker_name_;
    	int_marker.scale = ARROW_SCALE_;

        // Arrow visualization
        visualization_msgs::msg::Marker arrow;
        arrow.ns = marker_namespace_;
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.scale.x = ARROW_SCALE_; // shaft length
        arrow.scale.y = ARROW_SCALE_ / 10.0; // shaft diameter
        arrow.scale.z = ARROW_SCALE_ / 10.0; // head diameter
        arrow.color.r = ARROW_COLOR_R_;
        arrow.color.g = ARROW_COLOR_G_;
        arrow.color.b = ARROW_COLOR_B_;
        arrow.color.a = 1.0;

	// Add the marker to im control
        im_control.markers.push_back(arrow);
	}
    else if (info.marker_name==frame_marker_name_){

        int_marker.name = frame_marker_name_;
        int_marker.scale = FRAME_SCALE_;

	// Offset to avoid superposition of the arrow and frame
	int_marker.pose.position.x = ARROW_TO_FRAME_OFFSET_X_;
        int_marker.pose.position.y = ARROW_TO_FRAME_OFFSET_Y_;
        int_marker.pose.position.z = ARROW_TO_FRAME_OFFSET_Z_;   

        // Axes visualization
        visualization_msgs::msg::Marker x_axis; 
        x_axis.ns = marker_namespace_;
        x_axis.type = visualization_msgs::msg::Marker::ARROW; // Default orientation is already along x-axis for arrows
        x_axis.scale.x = FRAME_SCALE_; // shaft length
        x_axis.scale.y = FRAME_SCALE_ / 10.0; // shaft diameter
        x_axis.scale.z = FRAME_SCALE_ / 10.0; // head diameter
        x_axis.color.r = 1.0;  // red
        x_axis.color.a = 1.0;  // opaque
        
        visualization_msgs::msg::Marker y_axis;
        y_axis = x_axis;
        y_axis.color.r = 0.0;
        y_axis.color.g = 1.0; // green
	// Helper to set orientation
        auto set_orientation = [](auto &pose, const Eigen::Quaterniond &q) {
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
        };
	set_orientation(y_axis.pose, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ())));
        
        visualization_msgs::msg::Marker z_axis;
        z_axis = x_axis;
        z_axis.color.b = 1.0; // blue
        z_axis.color.r = 0.0;
	set_orientation(z_axis.pose, Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitY())));

	// Add the markers to im control
        im_control.markers.push_back(x_axis);
        im_control.markers.push_back(y_axis);
        im_control.markers.push_back(z_axis);

    }

    // Add the control to the interactive marker
    int_marker.controls.push_back(im_control);

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
        if (info.marker_name==arrow_marker_name_){
        server_->insert(int_marker,
                        std::bind(&InteractiveMarkerManager::process_arrow_feedback_, this, std::placeholders::_1));
	}
	else if (info.marker_name==frame_marker_name_){
        server_->insert(int_marker,
                        std::bind(&InteractiveMarkerManager::process_frame_feedback_, this, std::placeholders::_1));
	}
	
    }
    else
    {
        server_->insert(int_marker);
    }
    server_->applyChanges();
}

void InteractiveMarkerManager::hide_im(const std::string &marker_name, const std::string& planning_group)
{
    // Disable interactive marker controls
    ImControlEnableInfo enable_info;
    enable_info.marker_name = marker_name;
    enable_info.enable = ImControlEnable::NONE;
    enable_im_controls(enable_info, planning_group);

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

void InteractiveMarkerManager::draw_ee_or_control_im(const std::string &axis, const std::string& planning_group)
{
    ImControlEnableInfo arrow_enable_info;
    arrow_enable_info.marker_name = arrow_marker_name_;
    arrow_enable_info.in_tool_frame = true;
    // Enable interactive marker for manual mode (with rotation control) and return
    if (axis == "Interactive Axis")
    {
        arrow_enable_info.enable = ImControlEnable::ROTATION;
        enable_im_controls(arrow_enable_info, planning_group);
        return;
    }

    // Enable the arrow with no interactive control
    arrow_enable_info.enable = ImControlEnable::NONE;
    enable_im_controls(arrow_enable_info, planning_group);

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

Eigen::Matrix4d InteractiveMarkerManager::get_frame_pose()
{
    Eigen::Matrix4d frame_pose;

    // Go with default location if the frame hasn't moved
    if (frame_pose_.hasNaN())
    {
	frame_pose = DEFAULT_FRAME_POSE_;
    }
    else
    {
	frame_pose = frame_pose_;
    }
    return frame_pose;
}

void InteractiveMarkerManager::publish_transform_(const std::string& parent_frame, const std::string& child_frame, const Eigen::Vector3d& translation)
{
    // Create the transform message
    geometry_msgs::msg::TransformStamped transform_stamped;

    // Set header details
    transform_stamped.header.stamp = rclcpp::Time(0);// static valid-for-all-time
    transform_stamped.header.frame_id = parent_frame;
    transform_stamped.child_frame_id = child_frame;

    // Set translation (x, y, z)
    transform_stamped.transform.translation.x = translation[0];
    transform_stamped.transform.translation.y = translation[1];
    transform_stamped.transform.translation.z = translation[2];

    // We assume orientation same as ee_frame_
    transform_stamped.transform.rotation.x = 0.0;
    transform_stamped.transform.rotation.y = 0.0;
    transform_stamped.transform.rotation.z = 0.0;
    transform_stamped.transform.rotation.w = 1.0;

    // Publish the transform
    tf_static_broadcaster_->sendTransform(transform_stamped);
}
} // namespace interactive_marker_manager
