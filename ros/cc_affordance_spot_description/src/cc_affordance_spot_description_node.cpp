#include "rclcpp/rclcpp.hpp"
#include <Eigen/Core>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcAffordancePlannerRos>(node_options);
    node->lookup_robot_ros_setup_info(); // lookup trajectory as name and joint states topic name from the global
                                         // parameter server

    // Construct buffer to lookup tf data for the tag
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
    tf2::Duration timeout(1s);
    tf2_ros::Buffer tf_buffer(clock, timeout, node);

    // Compose affordance screw
    /*------------------------------------------------------------*/
    const bool aff_from_tag = false;
    /*------------------------------------------------------------*/
    Eigen::Matrix<double, 6, 1> aff_screw;
    Eigen::Vector3d w_aff; // screw axis
    Eigen::Vector3d q_aff; // location vector

    if (aff_from_tag)
    {
        /*------------------------------------------------------------*/
        const std::string affordance_frame = "affordance_frame"; // Name of the tag frame
        const std::string reference_frame = "arm0_base_link";    // Maybe we can move the aff_from_tag portion inside
                                                                 // the class later since reference_frame info is
                                                                 // available there.
        w_aff << -1, 0, 0;                                       // screw axis
        /*------------------------------------------------------------*/

        // Extract Affordance frame location from TF data
        const Eigen::Isometry3d tag_htm = AffordanceUtilROS::get_htm(reference_frame, affordance_frame, tf_buffer);
        q_aff = tag_htm.translation();
        RCLCPP_INFO_STREAM(node->get_logger(), "Here is the affordance frame location. Ensure it makes sense:\n"
                                                   << q_aff);
    }
    else
    {
        /*------------------------------------------------------------*/
        w_aff << 0, 0, 1;
        q_aff << 0, 0, 0;
        /*------------------------------------------------------------*/
    }

    // Compute the 6x1 screw vector
    aff_screw = AffordanceUtil::get_screw(w_aff, q_aff); // compute affordance screw

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

    // Run the planner
    node->run_cc_affordance_planner(aff_screw, aff_goal);
    /*------------------------------------------------------------*/
    // Optionally, with planner parameters call:
    /* node->run_cc_affordance_planner(aff_screw, aff_goal, aff_step, gripper_control_par_tau, accuracy); */
    /*------------------------------------------------------------*/

    // Call the function again with new (or old) aff_screw and aff_goal to execute
    // another affordance in series
    /* node->run_cc_affordance_planner(aff_screw, aff_goal); */

    /* rclcpp::spin(node); // Keep node alive */
    while (rclcpp::ok())
    {
    }

    return 0;
}
