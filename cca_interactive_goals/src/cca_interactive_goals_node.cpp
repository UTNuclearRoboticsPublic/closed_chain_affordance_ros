#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

class InteractiveMarkerNode : public rclcpp::Node
{
public:
    InteractiveMarkerNode()
    : Node("interactive_marker_node")
    {
        // Correct usage of InteractiveMarkerServer constructor
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("simple_marker", this->get_node_base_interface(), this->get_node_clock_interface(), this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

        // Create an interactive marker
        visualization_msgs::msg::InteractiveMarker int_marker;
        int_marker.header.frame_id = "world";
        int_marker.name = "my_marker";
        int_marker.description = "Simple 1-DOF Control";
        int_marker.pose.position.x = 0.0;
        int_marker.pose.position.y = 0.0;
        int_marker.pose.position.z = 0.0;

        // Create a control for the marker
        visualization_msgs::msg::InteractiveMarkerControl control;
        control.always_visible = true;

        // Create a box marker to visualize
        visualization_msgs::msg::Marker box_marker;
        box_marker.type = visualization_msgs::msg::Marker::CUBE;
        box_marker.scale.x = 0.1;
        box_marker.scale.y = 0.1;
        box_marker.scale.z = 0.1;
        box_marker.color.r = 0.5;
        box_marker.color.g = 0.5;
        box_marker.color.b = 0.5;
        box_marker.color.a = 1.0;

        // Add the box to the control
        control.markers.push_back(box_marker);

        // Add the control to the interactive marker
        int_marker.controls.push_back(control);

        // Insert the marker into the server and apply changes
        server_->insert(int_marker, std::bind(&InteractiveMarkerNode::processFeedback, this, std::placeholders::_1));
        server_->applyChanges();
    }

private:
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback from marker '%s'", feedback->marker_name.c_str());
    }

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InteractiveMarkerNode>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}