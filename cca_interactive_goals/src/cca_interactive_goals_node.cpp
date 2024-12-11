#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class InteractiveMarkerNode : public rclcpp::Node
{
public:
    InteractiveMarkerNode()
    : Node("interactive_marker_node")
    {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("simple_marker", this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        createArrowInteractiveMarker();
        createInvisibleInteractiveMarker();
        server_->applyChanges();
    }

private:
    void createArrowInteractiveMarker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
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

        // Create a menu control
        visualization_msgs::msg::InteractiveMarkerControl menu_control;
        menu_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MENU;
        menu_control.name = "motion_type_menu";
        menu_control.description = "Type";
        int_marker.controls.push_back(menu_control);

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

        // Set up type entries for menu
        menu_handler_.insert("Revolute", std::bind(&InteractiveMarkerNode::menuCallback, this, std::placeholders::_1));
        menu_handler_.insert("Translational", std::bind(&InteractiveMarkerNode::menuCallback, this, std::placeholders::_1));
        menu_handler_.insert("Screw", std::bind(&InteractiveMarkerNode::menuCallback, this, std::placeholders::_1));

        // Apply menu to the interactive marker
        menu_handler_.apply(*server_, int_marker.name);
    }

    void createInvisibleInteractiveMarker()
    {
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "map";
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

        server_->insert(int_marker, std::bind(&InteractiveMarkerNode::processInvisibleMarkerFeedback, this, std::placeholders::_1));
    }

    void processArrowFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        switch (feedback->event_type)
        {
            case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
                RCLCPP_INFO(this->get_logger(), "Arrow marker moved to position (%.2f, %.2f, %.2f) and orientation (%.2f, %.2f, %.2f, %.2f)",
                            feedback->pose.position.x,
                            feedback->pose.position.y,
                            feedback->pose.position.z,
                            feedback->pose.orientation.x,
                            feedback->pose.orientation.y,
                            feedback->pose.orientation.z,
                            feedback->pose.orientation.w);
                
                // Update the marker's pose
                server_->setPose(feedback->marker_name, feedback->pose);
                server_->applyChanges();
                break;

            case visualization_msgs::msg::InteractiveMarkerFeedback::MENU_SELECT:
                RCLCPP_INFO(this->get_logger(), "Type %d selected", feedback->menu_entry_id);
                break;
        }
    }

    void processInvisibleMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
        {
            RCLCPP_INFO(this->get_logger(), "Invisible marker moved to position (%.2f, %.2f, %.2f) and orientation (%.2f, %.2f, %.2f, %.2f)",
                        feedback->pose.position.x,
                        feedback->pose.position.y,
                        feedback->pose.position.z,
                        feedback->pose.orientation.x,
                        feedback->pose.orientation.y,
                        feedback->pose.orientation.z,
                        feedback->pose.orientation.w);
            
            // Update the marker's pose
            server_->setPose(feedback->marker_name, feedback->pose);
            server_->applyChanges();

            // Publish TF frame
            geometry_msgs::msg::TransformStamped transform;
            transform.header.stamp = this->now();
            transform.header.frame_id = "map";
            transform.child_frame_id = "approach_frame";
            transform.transform.translation.x = feedback->pose.position.x;
            transform.transform.translation.y = feedback->pose.position.y;
            transform.transform.translation.z = feedback->pose.position.z;
            transform.transform.rotation = feedback->pose.orientation;

            tf_broadcaster_->sendTransform(transform);
        }
    }

    void menuCallback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
    std::string new_header;
    switch (feedback->menu_entry_id)
    {
        case 1:
        new_header = "Revolute";
        menu_handler_.setCheckState(1, interactive_markers::MenuHandler::CHECKED);
        menu_handler_.setCheckState(2, interactive_markers::MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(3, interactive_markers::MenuHandler::UNCHECKED);
        break;
        case 2:
        new_header = "Translational";
        menu_handler_.setCheckState(1, interactive_markers::MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(2, interactive_markers::MenuHandler::CHECKED);
        menu_handler_.setCheckState(3, interactive_markers::MenuHandler::UNCHECKED);
        break;
        case 3:
        new_header = "Screw";
        menu_handler_.setCheckState(1, interactive_markers::MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(2, interactive_markers::MenuHandler::UNCHECKED);
        menu_handler_.setCheckState(3, interactive_markers::MenuHandler::CHECKED);
        break;
    }
    
    // Apply changes to the server
    menu_handler_.reApply(*server_);
    server_->applyChanges();
    
    RCLCPP_INFO(this->get_logger(), "Selected: %s", new_header.c_str());
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