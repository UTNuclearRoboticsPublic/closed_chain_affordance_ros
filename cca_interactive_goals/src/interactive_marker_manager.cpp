#include "cca_interactive_goals/interactive_marker_manager.hpp"

namespace interactive_marker_manager
{

InteractiveMarkerManager::InteractiveMarkerManager(const std::string& node_name): rclcpp::Node(node_name)
{

       	// server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("interactive_goals", this);
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "interactive_goals", this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

  enableInteractiveMarkerControls("arrow_marker", ImControlEnable::ALL, true);

}

void InteractiveMarkerManager::processArrowFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:

      // Update affordance axis and location in the planning request
      Eigen::Quaterniond q(feedback->pose.orientation.w,
          		feedback->pose.orientation.x,
          		feedback->pose.orientation.y,
          		feedback->pose.orientation.z);
      Eigen::Vector3d X_AXIS(1.0, 0.0, 0.0);//Starts as oriented along x
      affordance_axis_ = q * X_AXIS;
      affordance_location_= Eigen::Vector3d(
            feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

      server_->setPose(feedback->marker_name, feedback->pose);
      server_->applyChanges();
      break;
  }
}

void InteractiveMarkerManager::enableInteractiveMarkerControls(const std::string& marker_name, const ImControlEnable& enable, bool create) {
  visualization_msgs::msg::InteractiveMarker int_marker;

  // Retrieve marker if not creating a new one
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
  arrow.id = 8;

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
    server_->insert(int_marker, std::bind(&InteractiveMarkerManager::processArrowFeedback, this, std::placeholders::_1));
  } else {
    server_->insert(int_marker);
  }
  server_->applyChanges();

  // Reset affordance pose
  affordance_axis_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  affordance_location_ = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
}

void InteractiveMarkerManager::hideInteractiveMarker(const std::string& marker_name)
{    
  // Disable interactive marker controls 
  enableInteractiveMarkerControls(marker_name, ImControlEnable::NONE);

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

void InteractiveMarkerManager::drawEeOrControlIm(int index)
{
    RCLCPP_INFO(this->get_logger(), "About to draw EE Or mode marker");
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
  enableInteractiveMarkerControls("arrow_marker", ImControlEnable::ROTATION); 
  return;}


  // Enable the arrow with no interactive control
  enableInteractiveMarkerControls("arrow_marker", ImControlEnable::NONE);

  // Reset arrow below
  visualization_msgs::msg::InteractiveMarker int_marker;
  server_->get("arrow_marker", int_marker);

  for (auto& control : int_marker.controls)
{
  if (!control.markers.empty())
  {
    for (auto& marker : control.markers)
    {
      if (marker.id == 8) // Found the arrow marker
      {
        RCLCPP_INFO(this->get_logger(), "Arrow Located");

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
      }
    }
  }
}

// Update the interactive marker with the new arrow orientation
server_->insert(int_marker);
server_->applyChanges();
    RCLCPP_INFO(this->get_logger(), "Drew EE Or mode marker");

}

affordance_util::ScrewInfo InteractiveMarkerManager::getAffordancePose_(const std::string planning_mode, const std::string axis_mode) {
    affordance_util::ScrewInfo screw_info;

    // Check if asked to look at the moved interactive marker
    if ((planning_mode == "In-Place End Effector Orientation Control") && (axis_mode != "Interactive Axis")){

    RCLCPP_INFO(this->get_logger(), "Getting affordance info");
    // Create an InteractiveMarker to retrieve data
    visualization_msgs::msg::InteractiveMarker int_marker;

    // Retrieve the marker 
    server_->get("arrow_marker", int_marker);

    // Search through all controls and markers to find the one with the correct ID
    for (auto& control : int_marker.controls) {
        for (auto& marker : control.markers) {
            if (marker.id == marker_id_) { 

                // Extract the marker pose (position and orientation)
                const geometry_msgs::msg::Pose& marker_pose = marker.pose;

                // Extract position and orientation from the marker pose
                Eigen::Vector3d affordance_location(
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
                Eigen::Vector3d affordance_axis = arrow_quaternion * default_affordance_axis_;

                // Populate the screw_info struct
                screw_info.axis = affordance_axis;
                screw_info.location = affordance_location;

                return screw_info;
            }
        }
    }
    }
    else {
  if (affordance_axis_.hasNaN() && affordance_location_.hasNaN()) {
    RCLCPP_WARN(this->get_logger(),
                 "Requested planning without having moved the affordance screw axis arrow. Going with its default location and orientation");
    screw_info.axis = default_affordance_axis_;
    screw_info.location = default_affordance_location_;
} else {
    RCLCPP_WARN(this->get_logger(),
                 "Interactive marker has moved");
    screw_info.axis = affordance_axis_;
    screw_info.location = affordance_location_;
}
    
    }

}
}
