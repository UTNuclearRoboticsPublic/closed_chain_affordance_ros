// Copied from Alex Navarro's Spot Reachability package
#include <eigen3/Eigen/Geometry>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

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

    return 0;
}
