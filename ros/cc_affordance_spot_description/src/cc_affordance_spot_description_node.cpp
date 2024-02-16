#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcAffordancePlannerRos>(node_options);

    /*------------------------------------------------------------*/
    // Specify affordance screw
    const Eigen::Vector3d aff_screw_axis(0, 0, 1);          // screw axis
    const Eigen::Vector3d aff_screw_axis_location(0, 0, 0); // location vector
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Set affordance goal
    const double aff_goal = 0.5 * M_PI; // Code
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Optionally set planner parameters
    /* const double &aff_step = 0.3; */
    /* const int &gripper_control_par_tau = 1; */
    /* const double &accuracy = 10.0 / 100.0; */
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Run the planner
    node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal);
    // Or if getting affordance screw location from apriltag
    /* const std::string apriltag_frame_name = "affordance_frame"; */
    /* node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal); Note screw axis is manually set
     * in this case as aff_screw_axis above. Just the location is gotten from Apriltag*/
    /*------------------------------------------------------------*/

    /*------------------------------------------------------------*/
    // Optionally, with planner parameters call:
    /* node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal, aff_step,
     * gripper_control_par_tau, accuracy); */
    // or
    /* node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal, aff_step, gripper_control_par_tau,
     * accuracy); */
    /*------------------------------------------------------------*/

    // Call the function again with new (or old) aff_screw_axis, aff_screw_axis_location and aff_goal to execute
    // another affordance in series
    /* node->run_cc_affordance_planner(aff_screw_axis, aff_screw_axis_location, aff_goal); */
    // or
    /* node->run_cc_affordance_planner(aff_screw_axis, apriltag_frame_name, aff_goal); */

    /* rclcpp::spin(node); // Keep node alive */
    while (rclcpp::ok())
    {
    }

    return 0;
}
