#ifndef INTERACTIVE_MARKER_NODE_HPP
#define INTERACTIVE_MARKER_NODE_HPP

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

class InteractiveMarkerNode : public rclcpp::Node
{
public:
    InteractiveMarkerNode();

private:
    void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

#endif // INTERACTIVE_MARKER_NODE_HPP