#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class InteractiveMarkerNode : public rclcpp::Node
{
public:
  InteractiveMarkerNode() : Node("interactive_marker_node")
  {
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
        "simple_marker", this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

    // server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("my_marker_server");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    createArrowInteractiveMarker();
    createInvisibleInteractiveMarker();
    server_->applyChanges();
  }

private:
  void createArrowInteractiveMarker()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.name = "arrow_marker";
    int_marker.description = "Movable-Arrow";
    int_marker.scale = 1.0;

    // Create arrow marker
    visualization_msgs::msg::Marker arrow;
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.scale.x = 1.0;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.r = 0.5;
    arrow.color.g = 0.5;
    arrow.color.b = 0.5;
    arrow.color.a = 1.0;

    // Create a control for the arrow
    visualization_msgs::msg::InteractiveMarkerControl arrow_control;
    arrow_control.always_visible = true;
    arrow_control.markers.push_back(arrow);
    int_marker.controls.push_back(arrow_control);

    // Create controls for movement and rotation
    visualization_msgs::msg::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server_->insert(int_marker, std::bind(&InteractiveMarkerNode::processArrowFeedback, this, std::placeholders::_1));
  }

  void createInvisibleInteractiveMarker()
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "end_effector_link";
    int_marker.name = "approach_frame";
    int_marker.description = "";
    int_marker.scale = 1.0;

    // Create controls for movement and rotation
    visualization_msgs::msg::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    server_->insert(int_marker,
                    std::bind(&InteractiveMarkerNode::processInvisibleMarkerFeedback, this, std::placeholders::_1));
  }

  void processArrowFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    switch (feedback->event_type)
    {
      case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
        RCLCPP_INFO(this->get_logger(),
                    "Arrow marker moved to position (%.2f, %.2f, %.2f) and orientation(%.2f, %.2f, %.2f, %.2f)",
                    feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z,
                    feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
                    feedback->pose.orientation.w);
        server_->setPose(feedback->marker_name, feedback->pose);
        server_->applyChanges();
        break;
      case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
        processMenuFeedback(feedback);
        break;
    }
  }

  void processMenuFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    uint32_t id = feedback->menu_entry_id;
    std::string action;

    // Determine the action based on the selected menu entry
    if (id >= 111 && id <= 115)
      action = "Translation";
    else if ((id >= 121 && id <= 124) || (id >= 131 && id <= 134))
      action = "Rotation";
    else if (id >= 3211 && id <= 3234)
      action = "EE Orientation";
    else if (id >= 411 && id <= 434)
      action = "Approach Planning";
    else
    {
      RCLCPP_INFO(this->get_logger(), "Unhandled menu entry: %d", id);
      // return;
    }

    // Perform the action
    RCLCPP_INFO(this->get_logger(), "Performing action: %s", action.c_str());
    // Add your specific action logic here

    // Update the menu to mark the selected option
    visualization_msgs::msg::InteractiveMarker int_marker;

    if (server_->get(feedback->marker_name, int_marker))
    {
      // Reset all menu entries to default color
      for (auto& entry : int_marker.menu_entries)
      {
        if (entry.id == id)
        {
          menu_handler_.setCheckState(entry.id, interactive_markers::MenuHandler::CHECKED);
        }
        else
        {
          menu_handler_.setCheckState(entry.id, interactive_markers::MenuHandler::UNCHECKED);
        }
      }

      // Update the interactive marker
      server_->insert(int_marker);
      server_->applyChanges();

      RCLCPP_INFO(this->get_logger(), "Menu option %d selected and marked", id);
    }
  }

  void
  processInvisibleMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
    {
      RCLCPP_INFO(this->get_logger(),
                  "Invisible marker moved to position (%.2f, %.2f, %.2f) and orientation(% .2f, % .2f, % .2f, % .2f) ",
                  feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z,
                  feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
                  feedback->pose.orientation.w);

      // Update the marker's pose
      server_->setPose(feedback->marker_name, feedback->pose);
      server_->applyChanges();

      // Publish TF frame
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->now();
      transform.header.frame_id = "end_effector_link";
      transform.child_frame_id = "approach_frame";
      transform.transform.translation.x = feedback->pose.position.x;
      transform.transform.translation.y = feedback->pose.position.y;
      transform.transform.translation.z = feedback->pose.position.z;
      transform.transform.rotation = feedback->pose.orientation;

      tf_broadcaster_->sendTransform(transform);
    }
  }

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InteractiveMarkerNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}