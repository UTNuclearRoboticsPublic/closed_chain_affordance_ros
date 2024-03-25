// Copied from Alex Navarro's Spot Reachability package
#include <Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <affordance_util/affordance_util.hpp>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

#define ROS_INFO(...) RCLCPP_INFO(this->get_logger(), __VA_ARGS__)

class SpotIKSolver : public rclcpp::Node
{
  public:
    SpotIKSolver() : rclcpp::Node("spot_ik_node") {}

    void init()
    {
        model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this->shared_from_this());
        kinematic_model_ = model_loader_->getModel();
        robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
        robot_state_->setToDefaultValues();

        ROS_INFO("Loaded model for Spot robot, with model frame %s", kinematic_model_->getModelFrame().c_str());
    }

    bool setRobotState(const Eigen::VectorXd &joint_positions)
    {
        try
        {
            std::unordered_map<std::string, double> joint_values = setup_robot_state_values(joint_positions);
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

    std::unordered_map<std::string, double> setup_robot_state_values(const Eigen::VectorXd joint_positions)
    {

        std::unordered_map<std::string, double> joint_values; // result

        // Populate the map from the vector (assuming order matches)
        for (int i = 0; i < joint_names_.size(); ++i)
        {
            joint_values[joint_names_[i]] = joint_positions[i];
        }

        return joint_values;
    }

  protected:
    robot_model_loader::RobotModelLoaderPtr model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;

  private:
    const std::vector<std::string> joint_names_ = {"arm0_shoulder_yaw", "arm0_shoulder_pitch", "arm0_elbow_pitch",
                                                   "arm0_elbow_roll",   "arm0_wrist_pitch",    "arm0_wrist_roll"};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SpotIKSolver>();
    node->init();
    std::thread spinner_thread([&node]() { rclcpp::spin(node); });

    // Set robot state from affordance start config
    Eigen::VectorXd aff_start_state(6);
    aff_start_state << 0.5, -1.2, 1.0, 0.7, -0.3, 2.1;

    if (node->setRobotState(aff_start_state))
    {
        std::cerr << "Error: Could not set robot state" << std::endl;
        return 1;
    }

    // Compute EE pose at start config
    Eigen::Isometry3d start_ee_htm_iso = node->forwardKinematics("arm0_tool0"); // Where is the base frame set for this?
    Eigen::Matrix4d start_ee_htm;
    start_ee_htm.block<3, 3>(0, 0) = start_ee_htm_iso.linear();
    start_ee_htm.block<3, 1>(0, 3) = start_ee_htm_iso.translation();

    // Compute cartesian trajectory from affordance start Pose using affordance screw exponential map
    // Compute affordance screw
    const Eigen::Vector3d aff_screw_axis(0, 0, 1);          // screw axis
    const Eigen::Vector3d aff_screw_axis_location(0, 0, 0); // location vector
    const Eigen::Matrix<double, 6, 1> aff_screw =
        AffordanceUtil::get_screw(aff_screw_axis, aff_screw_axis_location); // affordance screw

    // Define affordance goal and step
    const double aff_goal = 0.5 * M_PI;
    double aff_step = 0.1;

    // Compute affordance twist
    Eigen::Matrix<double, 6, 1> aff_twist = aff_screw * aff_step;

    // Compute of max number of loop iterations based on affordance goal and affordance step
    const int stepper_max_itr_m = aff_goal / aff_step + 1;

    // Initialize loop condition parameters
    int loop_counter_k = 0;
    bool success = true;

    // Initialize solution
    std::vector<Eigen::VectorXd> solution;

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
        Eigen::Matrix4d ee_htm = AffordanceUtil::MatrixExp6(se3mat) * start_ee_htm;
        Eigen::Isometry3d ee_htm_iso;
        ee_htm_iso.linear() = ee_htm.block<3, 3>(0, 0);
        ee_htm_iso.translation() = ee_htm.block<3, 1>(0, 3);

        // Solve IK for the cartesian trajectory updating the seed sequentially
        std::optional<std::vector<double>> ik_result = node->inverseKinematics("arm", ee_htm_iso);

        if (ik_result.has_value())
        {
            Eigen::VectorXd point = Eigen::Map<Eigen::VectorXd>(ik_result.value().data(), ik_result.value().size());
            solution.push_back(point);

            // Update robot state
            success = node->setRobotState(point);

            // Update the start ee htm
            start_ee_htm = ee_htm;
        }
        else
        {
            std::cerr << "Error: IK wasn't successful." << std::endl;
            break;
        }
    }

    // Call moveit_plan_and_viz trajectory to visualize the plan

    rclcpp::shutdown();    // shutdown ROS
    spinner_thread.join(); // join the spinner thread
    return 0;
}
