// Credits: The following is directly copied from roboticsbackend ROS2 Global Parameters tutorial. This is a workaround
// to load parameters onto a ROS2 global parameter server for use from other nodes. Currently, ROS2 requires a node
// argument to load parameters. That node we are calling "global_parameter_server".
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(true);
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("global_parameter_server", options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
