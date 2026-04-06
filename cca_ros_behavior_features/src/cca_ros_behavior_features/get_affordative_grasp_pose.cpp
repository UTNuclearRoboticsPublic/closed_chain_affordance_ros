///////////////////////////////////////////////////////////////////////////////
//      Title     : get_affordative_grasp_pose.cpp
//      Project   : cca_ros_behavior_features
//      Created   : 2026
//      Author    : Crasun Jans
///////////////////////////////////////////////////////////////////////////////

#include "cca_ros_behavior_features/get_affordative_grasp_pose.hpp"

namespace cca_ros_behavior_features
{

GetAffordativeGraspPose::GetAffordativeGraspPose(const std::string &name, const BT::NodeConfig &config)
    : BT::StatefulActionNode(name, config)
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

BT::NodeStatus GetAffordativeGraspPose::onStart()
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    using ReqPtr = std::shared_ptr<cca_ros::PlanningRequest>;
    using PoseArrayPtr = std::shared_ptr<geometry_msgs::msg::PoseArray>;

    auto wbc_approach_req_exp = getInput<ReqPtr>("wbc_approach_req");
    if (!wbc_approach_req_exp)
        throw BT::RuntimeError("Missing input [wbc_approach_req]: ", wbc_approach_req_exp.error());

    auto arm_approach_req_exp = getInput<ReqPtr>("arm_approach_req");
    if (!arm_approach_req_exp)
        throw BT::RuntimeError("Missing input [arm_approach_req]: ", arm_approach_req_exp.error());

    auto arm_grab_req_exp = getInput<ReqPtr>("arm_grab_req");
    if (!arm_grab_req_exp)
        throw BT::RuntimeError("Missing input [arm_grab_req]: ", arm_grab_req_exp.error());

    auto grasp_poses_exp = getInput<PoseArrayPtr>("grasp_poses");
    if (!grasp_poses_exp)
        throw BT::RuntimeError("Missing input [grasp_poses]: ", grasp_poses_exp.error());

    auto wbc_req = wbc_approach_req_exp.value();
    auto arm_req = arm_approach_req_exp.value();
    auto grab_req = arm_grab_req_exp.value();
    auto poses = grasp_poses_exp.value();

    result_future_ = std::async(std::launch::async, [wbc_req, arm_req, grab_req, poses, this]() {
        return cca_ros_features::getAffordativeGraspPose(*wbc_req, *arm_req, *grab_req, *poses, timeout_,
                                                         arm_start_index_in_wbc_traj_, arm_num_joints_);
    });

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetAffordativeGraspPose::onRunning()
{
    if (result_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
        return BT::NodeStatus::RUNNING;

    const auto result = result_future_.get();
    if (!result)
    {
        RCLCPP_ERROR(node_->get_logger(), "No affordative grasp pose found");
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::PoseStamped stamped = *result;
    stamped.header.stamp = node_->now();
    setOutput("affordative_grasp_pose", std::make_shared<geometry_msgs::msg::PoseStamped>(stamped));
    return BT::NodeStatus::SUCCESS;
}

void GetAffordativeGraspPose::onHalted()
{
    if (result_future_.valid())
        result_future_.wait();
}

} // namespace cca_ros_behavior_features
