// Copied from Alex Navarro's Spot Reachability package
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

    bool setRobotState(const std::unordered_map<std::string, double> &joint_values)
    {
        try
        {
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

  protected:
    robot_model_loader::RobotModelLoaderPtr model_loader_;
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr robot_state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<SpotIKSolver>();
    node->init();

    // Set robot state from affordance start config
    // Define the joint names (optional)
    std::vector<std::string> joint_names = {"arm0_shoulder_yaw", "arm0_shoulder_pitch", "arm0_elbow_pitch",
                                            "arm0_elbow_roll",   "arm0_wrist_pitch",    "arm0_wrist_roll"};

    // Create a sample Eigen::VectorXd (modify with your actual data)
    Eigen::VectorXd joint_positions(6);
    joint_positions << 0.5, -1.2, 1.0, 0.7, -0.3, 2.1; // Replace with your joint values

    // Create the unordered map
    std::unordered_map<std::string, double> joint_values;

    // Check if joint names and vector size match
    if (joint_names.size() != joint_positions.size())
    {
        std::cerr << "Error: Number of joint names and positions do not match!" << std::endl;
        return 1;
    }

    // Populate the map from the vector (assuming order matches)
    for (int i = 0; i < joint_names.size(); ++i)
    {
        joint_values[joint_names[i]] = joint_positions[i];
    }

    if (node->setRobotState(joint_values))
    {
        // Compute FK from affordance start config
        Eigen::Isometry3d start_ee_htm = node->forwardKinematics("arm0_tool0"); // Where is the base frame set for this?

        // Compute cartesian trajectory from affordance start Pose using affordance screw exponential map
        // Specify affordance screw
        const Eigen::Vector3d aff_screw_axis(0, 0, 1);          // screw axis
        const Eigen::Vector3d aff_screw_axis_location(0, 0, 0); // location vector
                                                                // Compute affordance screw
        const Eigen::Matrix<double, 6, 1> aff_screw =
            AffordanceUtil::get_screw(aff_screw_axis, aff_screw_axis_location); // compute affordance screw
        double theta_step = 0.1;
        const Eigen::Matrix<double, 6, 1> twist = aff_screw * theta_step;

        Eigen::Matrix4d new_ee_htm = AffordanceUtil::MatrixExp6(AffordanceUtil::VecTose3(twist));

        // Solve IK for the cartesian trajectory updating the seed sequentially

        // Call moveit_plan_and_viz trajectory to visualize the plan
    }
    else
    {
        std::cerr << "Error: Could not set robot state" << std::endl;
        return 1;
    }

    return 0;
}
