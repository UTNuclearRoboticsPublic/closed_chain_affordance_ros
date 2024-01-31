#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cmath>
#include <future>
#include <moveit_plan_and_viz/srv/move_it_plan_and_viz.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>

using namespace std::chrono_literals;

class CcAffordancePlannerRos : public rclcpp::Node
{
  public:
    // Namespaces
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveItPlanAndViz = moveit_plan_and_viz::srv::MoveItPlanAndViz;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
    using JointState = sensor_msgs::msg::JointState;

    // Constructor
    explicit CcAffordancePlannerRos(const rclcpp::NodeOptions &options, const std::string &robot_config_file_path)
        : Node("cc_affordance_planner_ros", options), // Use new class name
          node_logger_(this->get_logger()),
          traj_execution_as_name_("/arm_controller/follow_joint_trajectory"),
          plan_and_viz_ss_name_("/moveit_plan_and_viz_server"),
          joint_states_topic_("/spot_driver/joint_states")
    {

        // Initialize clients and subscribers
        traj_execution_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, traj_execution_as_name_);
        plan_and_viz_client_ = this->create_client<MoveItPlanAndViz>(plan_and_viz_ss_name_);
        joint_states_sub_ = this->create_subscription<JointState>(
            joint_states_topic_, 1000,
            std::bind(&CcAffordancePlannerRos::joint_states_cb_, this, std::placeholders::_1));

        // Extract robot config info
        const AffordanceUtil::RobotConfig &robotConfig = AffordanceUtil::robot_builder(robot_config_file_path);
        robot_slist_ = robotConfig.Slist;
        M_ = robotConfig.M;
        tool_name_ = robotConfig.tool_name;
        joint_names_ = robotConfig.joint_names;
    }

    void run_cc_affordance_planner(const Eigen::VectorXd &aff_screw, const double &aff_goal)
    {

        // Get joint states at the start configuration of the affordance
        Eigen::VectorXd robot_thetalist = get_aff_start_joint_states_();
        /* Eigen::VectorXd robot_thetalist(joint_names_.size()); */
        /* robot_thetalist << -0.00076, -0.87982, 1.73271, 0.01271, -1.13217, -0.00273; // Pulling a drawer */
        /* robot_thetalist << 0.00795, -1.18220, 2.46393, 0.02025, -1.32321, */
        /*     -0.00053; // Pushing a drawer */
        /* robot_thetalist << 0.20841, -0.52536, 1.85988, 0.18575, -1.37188, -0.07426; // Moving a stool */
        /* robot_thetalist << 0.08788, -1.33410, 2.14567, 0.19725, -0.79857, 0.46613; // Turning a valve2 */
        /* robot_thetalist << 0.03456, -1.40627, 2.10997, 0.13891, -0.66079, 0.76027; // Turning a valve4 */

        std::cout << "Here are the captured joint states: \n" << joint_states_.positions << std::endl;

        // Compose cc model and affordance goal
        Eigen::MatrixXd cc_slist = AffordanceUtil::compose_cc_model_slist(robot_slist_, robot_thetalist, M_, aff_screw);
        std::cout << "Here is the full cc slist: \n" << cc_slist << std::endl;

        // Construct the CcAffordancePlanner object
        CcAffordancePlanner ccAffordancePlanner;

        // Set planner parameters
        auto sign_of = [](double x) {
            return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
        }; // Helper lambda to check the sign of affordance goal

        /* const double aff_step = 0.05; // Pulling a drawer */
        /* const double aff_step = 0.05;         // Pushing a drawer */
        const double aff_step = 0.3; // Moving a stool
        /* const double aff_step = 0.2;          // Turning a valve2 */
        /* const double aff_step = 0.2;          // Turning a valve4 */
        const double accuracy = 10.0 / 100.0; //

        ccAffordancePlanner.p_aff_step_deltatheta_a = sign_of(aff_goal) * aff_step;
        ccAffordancePlanner.p_task_err_threshold_eps_s = accuracy * aff_step;

        // Run the planner by passing the screw list, affordance goal, and task
        // offset
        /* const int gripper_control_par_tau = 1; // Pulling a drawer */
        /* const int gripper_control_par_tau = 1; // Pushing a drawer */
        const int gripper_control_par_tau = 1; // Moving a stool
        /* const int gripper_control_par_tau = 3; // Turning a valve2 */
        /* const int gripper_control_par_tau = 3; // Turning a valve4 */
        PlannerResult plannerResult =
            ccAffordancePlanner.affordance_stepper(cc_slist, aff_goal, gripper_control_par_tau);

        // Print the first point in the trajectory if planner succeeds
        std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
        if (plannerResult.success)
        {
            std::cout << "Planner succeeded with " << plannerResult.traj_full_or_partial
                      << " solution, and planning took " << plannerResult.planning_time.count() << " microseconds"
                      << std::endl;
            /* std::cout << "Here are the start and end affordances, " */
            /*           << "Start: " << solution[0](9) << ", End: " << solution[solution.size() - 1](9) << std::endl;
             */
            /* std::cout << "Here is the full solution:" << std::endl; */
            /* for (const auto &point : solution) */
            /* { */
            /*     std::cout << point << "\n\n"; */
            /* } */
        }
        else
        {
            std::cout << "Planner did not find a solution" << std::endl;
        }

        // Visualize and execute trajectory
        visualize_and_execute_trajectory_(solution);
    }

  private:
    rclcpp::Logger node_logger_; // logger associated with the node
    std::string traj_execution_as_name_;
    std::string plan_and_viz_ss_name_;
    std::string joint_states_topic_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr traj_execution_client_;
    rclcpp::Client<MoveItPlanAndViz>::SharedPtr plan_and_viz_client_; // Service client to visualize joint trajectory
    rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;    // Joint states subscriber

    // Robot data
    Eigen::MatrixXd robot_slist_;
    std::vector<std::string> joint_names_;
    Eigen::MatrixXd M_;
    std::string tool_name_;

    AffordanceUtilROS::JointTrajPoint joint_states_; // Processed and ordered joint states data

    void visualize_and_execute_trajectory_(std::vector<Eigen::VectorXd> trajectory)
    {
        // Visualize trajectory in RVIZ
        // Convert the solution trajectory to ROS message type
        const double traj_time_step = 0.3;
        const control_msgs::action::FollowJointTrajectory_Goal goal =
            AffordanceUtilROS::follow_joint_trajectory_msg_builder(
                trajectory, joint_states_.positions, joint_names_,
                traj_time_step); // this function takes care of extracting the right
                                 // number of joint_states although solution
                                 // contains qs data too

        auto plan_and_viz_serv_req = std::make_shared<MoveItPlanAndViz::Request>();
        plan_and_viz_serv_req->joint_traj = goal.trajectory;

        // Call service to visualize
        while (!plan_and_viz_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_logger_, "Interrupted while waiting for %s service. Exiting.",
                             plan_and_viz_ss_name_.c_str());
                return;
            }
            RCLCPP_INFO(node_logger_, plan_and_viz_ss_name_.c_str(), "service not available, waiting again...");
        }

        auto result = plan_and_viz_client_->async_send_request(plan_and_viz_serv_req);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node_logger_, plan_and_viz_ss_name_.c_str(), " service succeeded");
        }
        else
        {
            RCLCPP_ERROR(node_logger_, plan_and_viz_ss_name_.c_str(), "service call failed");
        }

        // Execute trajectory on the real robot
        std::string execution_conf;
        std::cout << "Ready to execute the trajectory? y to confirm" << std::endl;
        std::cin >> execution_conf;

        if (execution_conf != "y" && execution_conf != "Y")
        {
            std::cout << "You indicated you are not ready to execute trajectory" << std::endl;
            return;
        }

        // Send the goal to follow_joint_trajectory action server for execution
        if (!this->traj_execution_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_logger_, " %s action server not available after waiting",
                         traj_execution_as_name_.c_str());
            rclcpp::shutdown();
        }

        RCLCPP_INFO(node_logger_, "Sending goal to %s action server", traj_execution_as_name_.c_str());

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_goal_response_callback_, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&CcAffordancePlannerRos::traj_execution_feedback_callback_,
                                                        this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_result_callback_, this, std::placeholders::_1);

        this->traj_execution_client_->async_send_goal(goal, send_goal_options);
    }

    // Callback function for the joint_states subscriber
    void joint_states_cb_(const JointState::SharedPtr msg)
    {
        joint_states_ = AffordanceUtilROS::get_ordered_joint_states(
            msg,
            joint_names_); // Takes care of filtering and ordering the joint_states
    }

    // Function to read robot joint states at the start of the affordance
    Eigen::VectorXd get_aff_start_joint_states_()
    {
        // Manually set joint_states for planning time computation purposes only
        joint_states_.positions.conservativeResize(joint_names_.size());

        // Capture initial configuration joint states
        std::string capture_joint_states_conf;
        std::cout << "Put the robot in the start configuration for affordance "
                     "execution. Done? y for yes."
                  << std::endl;
        std::cin >> capture_joint_states_conf;

        if (capture_joint_states_conf != "y" && capture_joint_states_conf != "Y")
        {
            throw std::runtime_error("You indicated you are not ready to capture joint states");
        }
        // Take a second to read callbacks to ensure proper capturing of joint
        // states data
        rclcpp::Rate loop_rate(4); // Create a rate object with 4 Hz frequency
        for (int i = 0; i < 4; ++i)
        {
            rclcpp::spin_some(this->get_node_base_interface()); // Process incoming messages and callbacks
            loop_rate.sleep();                                  // Sleep to maintain the desired loop rate
        }

        return joint_states_.positions;
    }

    void traj_execution_feedback_callback_(GoalHandleFollowJointTrajectory::SharedPtr,
                                           const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        // Empty for now
    }

    void traj_execution_result_callback_(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_logger_, "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_logger_, "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(node_logger_, "Unknown result code");
            return;
        }
        RCLCPP_INFO(node_logger_, traj_execution_as_name_.c_str(), "action server call concluded");
        rclcpp::shutdown();
    }

    void traj_execution_goal_response_callback_(std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(node_logger_, "Goal was rejected by %s action server", traj_execution_as_name_.c_str());
        }
        else
        {
            RCLCPP_INFO(node_logger_, "Goal accepted by %s action server, waiting for result",
                        traj_execution_as_name_.c_str());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Find CC Affordance Description for the robot
    const std::string package_name = "cc_affordance_spot_description";
    const std::string rel_dir = "/config/";
    const std::string filename = package_name + ".yaml";
    std::string robot_cc_description_path;
    robot_cc_description_path = AffordanceUtilROS::get_filepath_inside_pkg(package_name, rel_dir, filename);

    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CcAffordancePlannerRos>(node_options, robot_cc_description_path);

    // Construct buffer to lookup tf data for the tag
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
    tf2::Duration timeout(1s);
    tf2_ros::Buffer tf_buffer(clock, timeout, node);

    // Compose affordance screw
    const bool aff_from_tag = false; // To be hard-coded as needed
    Eigen::Matrix<double, 6, 1> aff_screw;

    if (aff_from_tag)
    {
        const std::string affordance_frame = "affordance_frame"; // Name of the tag frame
        const std::string reference_frame = "arm0_base_link";    // Maybe we can move the aff_from_tag portion inside
                                                                 // the class later since reference_frame info is
                                                                 // available there.
        // Set screw axis
        const Eigen::Vector3d w_aff(-1, 0, 0); // To be hard-coded as needed

        // Extract Affordance frame location from TF data
        const Eigen::Isometry3d tag_htm = AffordanceUtilROS::get_htm(reference_frame, affordance_frame, tf_buffer);
        const Eigen::Vector3d q_aff = tag_htm.translation();
        std::cout << "Here is the affordance frame location. Ensure it makes sense: \n" << q_aff << std::endl;

        // Compute the 6x1 screw vector
        aff_screw = AffordanceUtil::get_screw(w_aff, q_aff); // return screw axis
    }
    else
    {
        const Eigen::Vector3d w_aff(0, 0, 1); // Moving a stool
        const Eigen::Vector3d q_aff(0, 0, 0); // Moving a stool
        /* const Eigen::Vector3d w_aff(-1, 0, 0);                       // Turning a valve2 */
        /* const Eigen::Vector3d q_aff(0.597133, -0.0887238, 0.170599); // Turning a valve2 */
        /* const Eigen::Vector3d w_aff(-1, 0, 0);                     // Turning a valve4 */
        /* const Eigen::Vector3d q_aff(0.602653, -0.119387, 0.16575); // Turning a valve4 */

        /* // Compute the 6x1 screw vector */
        aff_screw = AffordanceUtil::get_screw(w_aff, q_aff);

        // Pure translation edit
        /* aff_screw = (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, -1, 0, 0).finished(); // Pulling a drawer */
        /* aff_screw = (Eigen::Matrix<double, 6, 1>() << 0, 0, 0, 1, 0, */
        /*              *0).finished(); // Pushing a drawer */
    }

    // Set affordance goal
    const double aff_goal = 0.5 * M_PI; // Moving a stool
                                        /* const double aff_goal = -1.5 * M_PI; // Turning a valve2 */
                                        /* const double aff_goal = -0.5 * M_PI; // Turning a valve4 */
                                        /* const double aff_goal = -0.29; // Pulling a drawer */
                                        /* const double aff_goal = 0.2; // Pushing a drawer*/

    // Run the planner
    node->run_cc_affordance_planner(aff_screw, aff_goal);

    // Call the function again with new (or old) aff_screw and aff_goal to execute
    // another affordance in series
    /* node->run_cc_affordance_planner(aff_screw, aff_goal); */

    rclcpp::shutdown();
    return 0;
}
