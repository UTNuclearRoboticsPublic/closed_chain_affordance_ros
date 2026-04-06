#include "cca_ros_behavior_features/get_affordative_grasp_pose.hpp"

namespace cca_ros_behavior_features
{
GetAffordativeGraspPose::GetAffordativeGraspPose(const std::string &name, const BT::NodeConfig &config)
    : BT::SyncActionNode(name, config), rclcpp::Node(name)
{
}

BT::PortsList GetAffordativeGraspPose::providedPorts()
{
    return {BT::InputPort<std::shared_ptr<cca_ros::PlanningRequest>>("wbc_approach_req"),
            BT::InputPort<std::shared_ptr<cca_ros::PlanningRequest>>("arm_approach_req"),
            BT::InputPort<std::shared_ptr<cca_ros::PlanningRequest>>("arm_grab_req"),
            BT::InputPort<std::shared_ptr<geometry_msgs::msg::PoseArray>>("grasp_poses"),
            BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("affordative_grasp_pose")};
}

BT::NodeStatus GetAffordativeGraspPose::tick()
{
    // Define type alias for readability
    using ReqPtr = std::shared_ptr<cca_ros::PlanningRequest>;
    using PoseArrayPtr = std::shared_ptr<geometry_msgs::msg::PoseArray>;

    // Retrieve inputs
    BT::Expected<ReqPtr> wbc_approach_req_exp = getInput<ReqPtr>("wbc_approach_req");
    BT::Expected<ReqPtr> arm_approach_req_exp = getInput<ReqPtr>("arm_approach_req");
    BT::Expected<ReqPtr> arm_grab_req_exp = getInput<ReqPtr>("arm_grab_req");
    BT::Expected<PoseArrayPtr> grasp_poses_exp = getInput<PoseArrayPtr>("grasp_poses");

    // Check inputs and log errors if any are missing
    if (!wbc_approach_req_exp)
    {
        RCLCPP_ERROR(this->get_logger(), "Missing input: wbc_approach_req. Error: %s",
                     wbc_approach_req_exp.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (!arm_approach_req_exp)
    {
        RCLCPP_ERROR(this->get_logger(), "Missing input: arm_approach_req. Error: %s",
                     arm_approach_req_exp.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (!arm_grab_req_exp)
    {
        RCLCPP_ERROR(this->get_logger(), "Missing input: arm_grab_req. Error: %s", arm_grab_req_exp.error().c_str());
        return BT::NodeStatus::FAILURE;
    }
    if (!grasp_poses_exp)
    {
        RCLCPP_ERROR(this->get_logger(), "Missing input: grasp_poses. Error: %s", grasp_poses_exp.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Unpack inputs
    ReqPtr wbc_approach_req_ptr = wbc_approach_req_exp.value();
    ReqPtr arm_approach_req_ptr = arm_approach_req_exp.value();
    ReqPtr arm_grab_req_ptr = arm_grab_req_exp.value();
    PoseArrayPtr grasp_poses = grasp_poses_exp.value();

    // Find affordative grasp pose
    const auto result = cca_ros_features::getAffordativeGraspPose(
        *wbc_approach_req_ptr,
        *arm_approach_req_ptr,
        *arm_grab_req_ptr,
        *grasp_poses,
        timeout_,
        arm_start_index_in_wbc_traj,
        arm_num_joints);

    if (!result)
    {
        RCLCPP_ERROR(this->get_logger(), "No affordative grasp pose found");
        return BT::NodeStatus::FAILURE;
    }

    // Stamp and set output
    geometry_msgs::msg::PoseStamped affordative_grasp_pose_stamped = *result;
    affordative_grasp_pose_stamped.header.stamp = this->now();
    setOutput("affordative_grasp_pose",
              std::make_shared<geometry_msgs::msg::PoseStamped>(affordative_grasp_pose_stamped));

    return BT::NodeStatus::SUCCESS;
}
} // namespace cca_ros_behavior_features
