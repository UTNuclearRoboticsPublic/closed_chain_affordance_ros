// Some class functions Copied from Alex Navarro's Spot Reachability package
#include <Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "cc_affordance_planner/cc_affordance_planner.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <affordance_util/affordance_util.hpp>
#include <affordance_util_ros/affordance_util_ros.hpp>
#include <chrono>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_plan_and_viz/srv/move_it_plan_and_viz.hpp>
#include <rclcpp/rclcpp.hpp>

#define ROS_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)

class SpotIKSolver : public rclcpp::Node
{
  public:
    // Namespaces
    using MoveItPlanAndViz = moveit_plan_and_viz::srv::MoveItPlanAndViz;
    rclcpp::Logger node_logger_; // logger associated with the node

    SpotIKSolver()
        : rclcpp::Node("spot_ik_node"),
          node_logger_(this->get_logger()),
          plan_and_viz_ss_name_("/moveit_plan_and_viz_server")
    {
    }

    void init()
    {
        model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this());
        kinematic_model_ = model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        robot_state_->setToDefaultValues();

        // Initialize visualization client
        plan_and_viz_client_ = this->create_client<MoveItPlanAndViz>(plan_and_viz_ss_name_);

        // Print robot info
        print_robot_info();
    }

    void print_robot_info()
    {
        ROS_INFO("Loaded model for %s robot, with model frame %s", kinematic_model_->getName().c_str(),
                 kinematic_model_->getModelFrame().c_str());
        auto joint_group_names = kinematic_model_->getJointModelGroupNames();
        ROS_INFO("The robot has the following joint groups and joints:");
        for (const auto &joint_group_name : joint_group_names)
        {
            const moveit::core::JointModelGroup *const joint_group =
                kinematic_model_->getJointModelGroup(joint_group_name);
            std::ostringstream oss;
            for (const auto &joint_name : joint_group->getJointModelNames())
            {
                oss << joint_name << ", ";
            }
            // Remove the trailing comma and space
            std::string joint_names = oss.str();
            if (!joint_names.empty())
            {
                joint_names = joint_names.substr(0, joint_names.size() - 2);
            }
            ROS_INFO("Joint group \"%s\" with joint names: %s", joint_group_name.c_str(), joint_names.c_str());
        }
    }

    bool setRobotState(const std::unordered_map<std::string, double> &joint_values)
    {
        try
        {
            /* std::unordered_map<std::string, double> joint_values = setup_robot_state_values(joint_positions); */
            for (const auto &[name, position] : joint_values)
            {
                robot_state_->setJointPositions(name, &position);
            }
            return true;
        }
        catch (std::out_of_range &e)
        {
            return false;
        }
    }

    Eigen::Isometry3d forwardKinematics(const std::string &link_name)
    {
        try
        {
            return robot_state_->getGlobalLinkTransform(link_name);
        }
        catch (moveit::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to calculate forward kinematics for unknown link %s", e.what());
            throw e;
        }
    }

    std::optional<std::vector<double>> inverseKinematics(const std::string &group, const Eigen::Isometry3d &ee_pose)
    {
        std::vector<double> result;
        const moveit::core::JointModelGroup *const joint_group = kinematic_model_->getJointModelGroup(group);
        if (robot_state_->setFromIK(joint_group, ee_pose, 0.1))
        {
            robot_state_->copyJointGroupPositions(joint_group, result);
            return result;
        }
        else
            return {};
    }

    /* std::optional<std::vector<Eigen::VectorXd>> call_cca_planner(const Eigen::MatrixXd &cc_slist, */
    /*                                                              const Eigen::VectorXd aff_start_state, */
    /*                                                              const Eigen::Matrix<double, 6, 1> &aff_screw, */
    /*                                                              const double &aff_goal, const double &aff_step, */
    /*                                                              const int &gripper_control_par_tau = 1, */
    /*                                                              const double &accuracy = 10.0 / 100.0) */
    std::optional<std::vector<Eigen::VectorXd>> call_cca_planner(
        const Eigen::MatrixXd &cc_slist, const Eigen::VectorXd aff_start_state,
        const Eigen::Matrix<double, 6, 1> &aff_screw, const Eigen::VectorXd &sec_goal, const double &aff_step,
        const int &gripper_control_par_tau = 1, const double &accuracy = 10.0 / 100.0)
    {

        const double aff_goal = sec_goal.tail(1)(0);
        // Construct the CcAffordancePlanner object
        CcAffordancePlanner ccAffordancePlanner;

        // Set planner parameters
        auto sign_of = [](double x) {
            return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
        }; // Helper lambda to check the sign of affordance goal

        ccAffordancePlanner.p_aff_step_deltatheta_a = sign_of(aff_goal) * abs(aff_step);
        /* ccAffordancePlanner.p_task_err_threshold_eps_s = accuracy * aff_step; */
        ccAffordancePlanner.p_accuracy = accuracy;
        std::cout << "Planner param aff step: " << ccAffordancePlanner.p_aff_step_deltatheta_a << std::endl;
        std::cout << "Planner param err accuracy: " << ccAffordancePlanner.p_accuracy << std::endl;

        // Call the planner
        /* PlannerResult plannerResult = */
        /*     ccAffordancePlanner.affordance_stepper(cc_slist, aff_goal, gripper_control_par_tau); */
        PlannerResult plannerResult =
            ccAffordancePlanner.affordance_stepper(cc_slist, sec_goal, gripper_control_par_tau);
        /* PlannerResult plannerResult = */
        /*     ccAffordancePlanner.affordance_stepper(cc_slist_body, aff_goal, gripper_control_par_tau); */

        // Print planner result
        std::vector<Eigen::VectorXd> cca_solution = plannerResult.joint_traj;
        if (plannerResult.success)
        {
            RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with " << plannerResult.traj_full_or_partial
                                                                       << " solution, and planning took "
                                                                       << plannerResult.planning_time.count()
                                                                       << " microseconds");
            std::cout << "\nHere is the entire trajectory:\n";
            for (const auto &point : cca_solution)
            {
                std::cout << point.transpose() << std::endl;
            }
            return cca_solution;
        }
        else
        {
            RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
            return {};
        }
    }

    std::optional<std::vector<Eigen::VectorXd>> call_standard_planner(
        const std::string &planning_group, const std::vector<std::string> &joint_names, const std::string &tool_frame,
        const Eigen::VectorXd aff_start_state, const Eigen::Matrix<double, 6, 1> &aff_screw, const double &aff_goal,
        double aff_step, const double &accuracy = 10.0 / 100.0)
    {

        // Set planner parameters
        auto sign_of = [](double x) {
            return (x > 0) ? 1.0 : (x < 0) ? -1.0 : 0.0;
        }; // Helper lambda to check the sign of affordance goal

        aff_step = sign_of(aff_goal) * abs(aff_step);
        auto start_time = std::chrono::high_resolution_clock::now(); // Monitor clock to track planning time

        // Set start state
        std::unordered_map<std::string, double> start_joint_state;

        for (size_t i = 0; i < joint_names.size(); ++i)
        {
            start_joint_state[joint_names[i]] =
                aff_start_state[i]; // assuming joint_names and aff_start_state elements are in the same order
        }

        if (!this->setRobotState(start_joint_state))
        {
            RCLCPP_ERROR(node_logger_, "Could not set affordance start config robot state");
            return {};
        }

        // Compute FK
        Eigen::Isometry3d start_ee_htm = this->forwardKinematics(tool_frame);

        // Compute affordance twist
        Eigen::Matrix<double, 6, 1> aff_twist = aff_screw * aff_step;

        // Compute of max number of loop iterations based on affordance goal and affordance step
        const int stepper_max_itr_m = aff_goal / aff_step + 1;

        std::cout << "MAX ITER:" << stepper_max_itr_m << std::endl;

        // Initialize loop condition parameters
        int loop_counter_k = 0;
        bool success = true;

        // Initialize solution
        std::vector<Eigen::VectorXd> trajectory;
        trajectory.push_back(aff_start_state); // store start config as first point in the solution

        while ((loop_counter_k < stepper_max_itr_m) && success)
        {
            // If last iteration, adjust affordance twist accordingly
            if (loop_counter_k == (stepper_max_itr_m))
            {
                aff_step = aff_goal - aff_step * (stepper_max_itr_m - 1);
                aff_twist = aff_screw * aff_step;
            }

            // Compute cartesian pose
            Eigen::Matrix4d se3mat = AffordanceUtil::VecTose3(aff_twist);
            Eigen::Matrix4d ee_htm_matrix = AffordanceUtil::MatrixExp6(se3mat) * start_ee_htm.matrix();
            /* std::cout << "The EE pose at step " << loop_counter_k << " is \n" << ee_htm << std::endl; */
            Eigen::Isometry3d ee_htm(ee_htm_matrix);

            // Solve IK for the cartesian trajectory updating the seed sequentially
            std::optional<std::vector<double>> ik_result = this->inverseKinematics(planning_group, ee_htm);
            std::cout << "Here is the start ee htm: \n" << ee_htm_matrix << std::endl;

            if (ik_result.has_value())
            {
                Eigen::VectorXd point = Eigen::Map<Eigen::VectorXd>(ik_result.value().data(), ik_result.value().size());
                trajectory.push_back(point);

                std::cout << "IK solution is: \n" << point << std::endl;
                // Update robot state
                std::unordered_map<std::string, double> joint_state;
                for (size_t i = 0; i < joint_names.size(); ++i)
                {
                    joint_state[joint_names[i]] =
                        point[i]; // assuming joint_names and point elements are in the same order
                }
                success = this->setRobotState(joint_state);

                // Update the start ee htm
                start_ee_htm = ee_htm;
            }
            else
            {
                std::cerr << "Error: IK wasn't successful at iteration " << loop_counter_k << std::endl;
                success = false;
                break;
            }
            loop_counter_k += 1;
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        auto planning_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

        std::cout << "Standard stepper planning time: " << planning_time.count() << " microseconds" << std::endl;

        if (success) // If it is 1 then, it just pushed the aff start state
        {
            std::cout << "Trajectory before visualization:\n";
            for (const auto &point : trajectory)
            {
                std::cout << point.transpose() << std::endl;
            }
            return trajectory;
        }
        else
        {
            std::cerr << "IK wasn't successful" << std::endl;

            return {};
        }
    }

    // Function to visualize planned trajectory
    bool visualize_trajectory(const std::vector<Eigen::VectorXd> &trajectory,
                              const std::vector<std::string> joint_names, const Eigen::VectorXd &config_offset)
    {

        // Convert the solution trajectory to ROS message type
        const double traj_time_step = 0.3;
        control_msgs::action::FollowJointTrajectory_Goal goal = AffordanceUtilROS::follow_joint_trajectory_msg_builder(
            trajectory, config_offset, joint_names,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains qs data too
        // Print the message
        // Loop through each trajectory point
        std::cout << "\nTrajectory Points:\n";
        for (const auto &point : goal.trajectory.points)
        {
            std::cout << "  Point " << (point.time_from_start.sec + point.time_from_start.nanosec * 1e-9)
                      << "s:" << std::endl;
            std::cout << "    Positions: [";
            for (double pos : point.positions)
            {
                std::cout << pos << ", ";
            }
            std::cout << "\b\b]\n"; // Remove trailing comma and space
        }
        std::cout << std::flush;
        // Fill out service request
        auto plan_and_viz_serv_req = std::make_shared<MoveItPlanAndViz::Request>();
        plan_and_viz_serv_req->joint_traj = goal.trajectory;
        plan_and_viz_serv_req->ref_frame = "arm0_base_link";
        plan_and_viz_serv_req->tool_frame = "arm0_tool0";
        plan_and_viz_serv_req->planning_group = "arm";
        plan_and_viz_serv_req->robot_description = "robot_description";
        plan_and_viz_serv_req->rviz_fixed_frame = "base_link";

        using namespace std::chrono_literals;
        // Call service to visualize
        while (!plan_and_viz_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(node_logger_, "Interrupted while waiting for %s service. Exiting.",
                             plan_and_viz_ss_name_.c_str());
                return false;
            }
            RCLCPP_INFO(node_logger_, " %s service not available, waiting again...", plan_and_viz_ss_name_.c_str());
        }

        auto result_future = plan_and_viz_client_->async_send_request(plan_and_viz_serv_req);
        RCLCPP_INFO(node_logger_, "Waiting on %s service to complete", plan_and_viz_ss_name_.c_str());
        auto response = result_future.get(); // blocks until response is received

        if (response->success)
        {
            RCLCPP_INFO(node_logger_, " %s service succeeded", plan_and_viz_ss_name_.c_str());
            // Execute trajectory on the real robot
            return true;
        }
        else
        {
            RCLCPP_ERROR(node_logger_, "%s service call failed", plan_and_viz_ss_name_.c_str());
            return false;
        }
    }

  private:
    robot_model_loader::RobotModelLoaderPtr model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;
    rclcpp::Client<MoveItPlanAndViz>::SharedPtr plan_and_viz_client_;
    std::string plan_and_viz_ss_name_; // name of the planning visualization server
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SpotIKSolver>();
    node->init();
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });

    /* const std::string planner = "standard"; */
    const std::string planner = "cca";

    // Extract robot config info
    const std::string robot_config_file_path = "/home/crasun/ws_moveit2/src/cca_spot/config/cca_spot_description.yaml";
    const std::string planning_group = "arm";
    const AffordanceUtil::RobotConfig &robotConfig = AffordanceUtil::robot_builder(robot_config_file_path);
    const Eigen::MatrixXd robot_slist = robotConfig.Slist;
    const Eigen::Matrix4d M = robotConfig.M;
    const std::vector<std::string> joint_names = robotConfig.joint_names;
    const std::string tool_frame = robotConfig.tool_name;
    std::cout << "Here is the robot screw list: \n" << robot_slist << std::endl;

    // Set robot state from affordance start config
    Eigen::VectorXd aff_start_state(6);
    /* aff_start_state << 0.20841, -0.52536, 1.85988, 0.18575, -1.37188, -0.07426; // moving a stool */
    aff_start_state << 0.08788, -1.33410, 2.14567, 0.19725, -0.79857, 0.46613; // turning a valve2
    /* aff_start_state << 0.00795, -1.18220, 2.46393, 0.02025, -1.32321, -0.00053; // pushing a drawer */
    /* aff_start_state << -0.00076, -0.87982, 1.73271, 0.01271, -1.13217, -0.00273; // pulling a drawer */
    /* aff_start_state = Eigen::VectorXd::Zero(6); */

    // Compute affordance screw
    /* const Eigen::Vector3d aff_screw_axis(0, 0, -1);         // screw axis - moving a stool */
    /* const Eigen::Vector3d aff_screw_axis_location(0, 0, 0); // location vector - moving a stool */
    const Eigen::Vector3d aff_screw_axis(-1, 0, 0); // screw axis - turning a valve 2
    const Eigen::Vector3d aff_screw_axis_location(0.597133, -0.0887238,
                                                  0.170599); // location vector - turning a valve 2
    const Eigen::Matrix<double, 6, 1> aff_screw =
        AffordanceUtil::get_screw(aff_screw_axis, aff_screw_axis_location); // affordance screw
    /* Eigen::VectorXd aff_screw(6); */
    /* aff_screw << 0, 0, 0, 1, 0, 0; // pushing a drawer */
    /* aff_screw << 0, 0, 0, 1, 0, 0; // pulling a drawer */

    // Define affordance goal and step
    /* const double aff_goal = 0.5 * M_PI; // moving a stool */
    /* double aff_step = 0.15;             // moving a stool */
    const double aff_goal = 4.0 * M_PI; // turning a valve2
    double aff_step = 0.2;              // turning a valve2
    /* const double aff_goal = 0.2; // pushing a drawer */
    /* double aff_step = 0.05;      // pushing a drawer */
    /* const double aff_goal = 0.29; // pulling a drawer */
    /* double aff_step = 0.05;       // pulling a drawer */
    /* const int gripper_control_par_tau = 1; // moving a stool */
    const int gripper_control_par_tau = 2; // turning a valve2
    /* const int gripper_control_par_tau = 3; // turning a valve2 */
    /* const int gripper_control_par_tau = 4; // turning a valve2 */
    /* const Eigen::Matrix<double, 1, 1> sec_goal(aff_goal); */
    const Eigen::Matrix<double, 2, 1> sec_goal(-0.2, aff_goal);
    /* const Eigen::Matrix<double, 3, 1> sec_goal(0.0, 0.0, aff_goal); */
    /* const Eigen::Matrix<double, 4, 1> sec_goal(0.0, 0.0, 0.0, aff_goal); */

    if (planner == "cca")
    {
        // Call CCA planner
        // Create closed-chain screws
        Eigen::MatrixXd cc_slist = AffordanceUtil::compose_cc_model_slist(robot_slist, aff_start_state, M, aff_screw);
        /* cc_slist.col(6) = -cc_slist.col(6); */
        /* std::cout << "\nHere is the space-frame screw list before swap: \n" << cc_slist << std::endl; */
        /* Eigen::VectorXd swapper_x = cc_slist.col(6); */
        /* Eigen::VectorXd swapper_y = cc_slist.col(7); */
        /* Eigen::VectorXd swapper_z = cc_slist.col(8); */
        /* std::cout << "swapper_x: " << swapper_x.transpose() << std::endl; */
        /* std::cout << "swapper_y: " << swapper_y.transpose() << std::endl; */
        /* std::cout << "swapper_z: " << swapper_z.transpose() << std::endl; */
        /* cc_slist.col(6) = swapper_y; */
        /* cc_slist.col(7) = swapper_z; */
        /* cc_slist.col(8) = swapper_x; */
        /* Eigen::MatrixXd new_cc_slist(cc_slist.rows(), cc_slist.cols() - 1); */
        /* new_cc_slist << cc_slist.leftCols(6), cc_slist.rightCols(3); */

        /* std::cout << "\nHere is the space-frame screw list: \n" << cc_slist << std::endl; */
        /* std::cout << "\nHere is the new space-frame screw list: \n" << new_cc_slist << std::endl; */
        /* std::optional<std::vector<Eigen::VectorXd>> planner_result = node->call_cca_planner( */
        /*     cc_slist, aff_start_state, aff_screw, aff_goal, aff_step, gripper_control_par_tau, 1.0 / 100.0); */
        std::optional<std::vector<Eigen::VectorXd>> planner_result = node->call_cca_planner(
            cc_slist, aff_start_state, aff_screw, sec_goal, aff_step, gripper_control_par_tau, 1.0 / 100.0);
        /* std::optional<std::vector<Eigen::VectorXd>> planner_result = node->call_cca_planner( */
        /*     new_cc_slist, aff_start_state, aff_screw, sec_goal, aff_step, gripper_control_par_tau, 1.0 / 100.0); */
        if (planner_result.has_value())
        {
            const Eigen::VectorXd config_offset = aff_start_state;
            /* auto planner_part_result = */
            /* std::vector<Eigen::VectorXd>(planner_result.value().begin(), planner_result.value().begin() + 7); */
            node->visualize_trajectory(planner_result.value(), joint_names, config_offset);
            /* node->visualize_trajectory(planner_part_result, joint_names, config_offset); */
        }
    }
    else if (planner == "standard")
    {
        // Call standard planner
        std::optional<std::vector<Eigen::VectorXd>> planner_result = node->call_standard_planner(
            planning_group, joint_names, tool_frame, aff_start_state, aff_screw, aff_goal, aff_step, 10.0 / 100.0);

        if (planner_result.has_value())
        {
            const Eigen::VectorXd config_offset = Eigen::VectorXd::Zero(aff_start_state.size());
            node->visualize_trajectory(planner_result.value(), joint_names, config_offset);
        }
    }
    else
    {
        RCLCPP_ERROR(node->node_logger_, "Wrong planner name");
    }

    rclcpp::shutdown();    // shutdown ROS
    spinner_thread.join(); // join the spinner thread
    return 0;
}
