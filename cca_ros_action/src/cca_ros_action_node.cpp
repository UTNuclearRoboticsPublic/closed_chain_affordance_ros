#include "cca_ros_action/cca_ros_action.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<cca_ros_action::CcaRosActionServer>("cca_ros_action", node_options);
    rclcpp::spin(node);
    return 0;
}
