#include "cca_interactive_goals/interactive_marker_manager.hpp"

namespace interactive_marker_manager
{

  // Initialize Eigen static consts
  const Eigen::Vector3d InteractiveMarkerManager::X_AXIS_(1.0, 0.0, 0.0);
  const Eigen::Vector3d InteractiveMarkerManager::DEFAULT_ARROW_AXIS_ = InteractiveMarkerManager::X_AXIS_;
  const Eigen::Vector3d InteractiveMarkerManager::DEFAULT_ARROW_LOCATION_(0.0, 0.0, 0.0);

InteractiveMarkerManager::InteractiveMarkerManager(const std::string& node_name): rclcpp::Node(node_name), arrow_marker_name_("arrow_marker")
{

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "interactive_goals", this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

  enable_im_controls(arrow_marker_name_, ImControlEnable::ALL, true);


  RCLCPP_INFO(this->get_logger(),"Interactive marker manager initialized.");
                 
}

void InteractiveMarkerManager::process_arrow_feedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:

      // Get orientation transform (this is wrt default x-axis)	    
      Eigen::Quaterniond q(feedback->pose.orientation.w,
          		feedback->pose.orientation.x,
          		feedback->pose.orientation.y,
          		feedback->pose.orientation.z);
      arrow_axis_ = q * X_AXIS_;
      arrow_location_= Eigen::Vector3d(
            feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

      server_->setPose(feedback->marker_name, feedback->pose);
      server_->applyChanges();
      break;
  }
}

void InteractiveMarkerManager::enable_im_controls(const std::string& marker_name, const ImControlEnable& enable, bool create) {
  visualization_msgs::msg::InteractiveMarker int_marker;

  // Retrieve marker if not create a new one
  if (!create) {
    server_->get(marker_name, int_marker);
  }

  // Clear and initialize marker
  int_marker = visualization_msgs::msg::InteractiveMarker();
  int_marker.header.frame_id = "arm0_base_link";
  int_marker.name = marker_name;
  int_marker.description = "";
  int_marker.scale = 0.5;

  // Lambda to add control
  auto addControl = [&](const std::string& name, double x, double y, double z, bool isRotation) {
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.orientation.x = x;
    control.orientation.y = y;
    control.orientation.z = z;
    control.name = name;
    control.interaction_mode = isRotation ? 
      visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS : 
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);
  };

  // Create and add arrow marker
  visualization_msgs::msg::Marker arrow;
  arrow.ns = "interactive_goals";
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.scale.x = 0.5;
  arrow.scale.y = 0.05;
  arrow.scale.z = 0.05;
  arrow.color.r = 0.251;
  arrow.color.g = 0.878;
  arrow.color.b = 0.816;
  arrow.color.a = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl arrow_control;
  arrow_control.always_visible = true;
  arrow_control.markers.push_back(arrow);
  int_marker.controls.push_back(arrow_control);

  // Helper to add rotation and translation controls
  auto addRotationControls = [&]() {
    addControl("rotate_x", 1.0, 0.0, 0.0, true);
    addControl("rotate_y", 0.0, 1.0, 0.0, true);
    addControl("rotate_z", 0.0, 0.0, 1.0, true);
  };

  auto addTranslationControls = [&]() {
    addControl("move_x", 1.0, 0.0, 0.0, false);
    addControl("move_y", 0.0, 1.0, 0.0, false);
    addControl("move_z", 0.0, 0.0, 1.0, false);
  };

  // Configure controls based on enable type
  switch (enable) {
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
  if (create) { //Insert with callback if it does not exist
    server_->insert(int_marker, std::bind(&InteractiveMarkerManager::process_arrow_feedback, this, std::placeholders::_1));
  } else {
    server_->insert(int_marker);
  }
  server_->applyChanges();

  // Reset arrow pose
  arrow_axis_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  arrow_location_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

void InteractiveMarkerManager::hide_im(const std::string& marker_name)
{    
  // Disable interactive marker controls 
  enable_im_controls(marker_name, ImControlEnable::NONE);

  // Get the interactive marker object
  visualization_msgs::msg::InteractiveMarker int_marker;
  server_->get(marker_name, int_marker);

  // Hide all non-interactive makers by making them fully transparent
  std::for_each(int_marker.controls.begin(), int_marker.controls.end(), [](auto& control) {
    std::for_each(control.markers.begin(), control.markers.end(), [](auto& marker) {
      marker.color.a = 0.0;  
    });
  });

  server_->insert(int_marker);
  server_->applyChanges();
}

void InteractiveMarkerManager::draw_ee_or_control_im(int index)
{
  enum class AxisOption {
    Manual = 1,
    X = 2,
    Y = 3,
    Z = 4,
    XMinus = 5,
    YMinus = 6,
    ZMinus = 7
};
  // Enable interactive marker for manual mode (with rotation control) and return
  if (static_cast<AxisOption>(index)==AxisOption::Manual){
  enable_im_controls(arrow_marker_name_, ImControlEnable::ROTATION); 
  return;}


  // Enable the arrow with no interactive control
  enable_im_controls(arrow_marker_name_, ImControlEnable::NONE);

  // Reset arrow below
  visualization_msgs::msg::InteractiveMarker int_marker;
  server_->get(arrow_marker_name_, int_marker);

    auto& marker = int_marker.controls.front().markers.front(); // First control contains the arrow marker

	switch (static_cast<AxisOption>(index)) {
	    case AxisOption::X:
		marker.pose.orientation.w = 1.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		break;

	    case AxisOption::Y:
		marker.pose.orientation.w = 0.707107;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.707107;
		break;

	    case AxisOption::Z:
		marker.pose.orientation.w = 0.707107;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = -0.707107;
		marker.pose.orientation.z = 0.0;
		break;

	    case AxisOption::XMinus:
		marker.pose.orientation.w = 0.0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 1.0;
		break;

	    case AxisOption::YMinus:
		marker.pose.orientation.w = 0.707107;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = -0.707107;
		break;

	    case AxisOption::ZMinus:
		marker.pose.orientation.w = 0.707107;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.707107;
		marker.pose.orientation.z = 0.0;
		break;

}

// Update the interactive marker with the new arrow orientation
server_->insert(int_marker);
server_->applyChanges();

}

affordance_util::ScrewInfo InteractiveMarkerManager::get_arrow_pose(const std::string planning_mode, const std::string axis_mode) {
    affordance_util::ScrewInfo screw_info;

    // Check if asked to look at the moved interactive marker
    if ((planning_mode == "In-Place End Effector Orientation Control") && (axis_mode != "Interactive Axis")){

    // Create an InteractiveMarker to retrieve data
    visualization_msgs::msg::InteractiveMarker int_marker;

    // Retrieve the marker 
    server_->get(arrow_marker_name_, int_marker);

    auto& marker = int_marker.controls.front().markers.front(); // First control contains the arrow marker
    
    // Extract the marker pose (position and orientation)
    const geometry_msgs::msg::Pose& marker_pose = marker.pose;

    // Extract position and orientation from the marker pose
    Eigen::Vector3d arrow_location(
        marker_pose.position.x,
        marker_pose.position.y,
        marker_pose.position.z
    );

    Eigen::Quaterniond arrow_quaternion(
        marker_pose.orientation.w,
        marker_pose.orientation.x,
        marker_pose.orientation.y,
        marker_pose.orientation.z
    );

    // Rotate the default axis using the quaternion from the marker's orientation
    Eigen::Vector3d arrow_axis = arrow_quaternion * DEFAULT_ARROW_AXIS_;

    // Populate the screw_info struct
    screw_info.axis = arrow_axis;
    screw_info.location = arrow_location;

    return screw_info;

    }
    else {
	    // Go with default location if the arrow hasn't moved
  if (arrow_axis_.hasNaN() && arrow_location_.hasNaN()) {
    screw_info.axis = DEFAULT_ARROW_AXIS_;
    screw_info.location = DEFAULT_ARROW_LOCATION_;
} else {
    screw_info.axis = arrow_axis_;
    screw_info.location = arrow_location_;
}
    
    }

}
}
