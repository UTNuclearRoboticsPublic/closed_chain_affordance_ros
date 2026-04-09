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
    if (!cca_ros_context_)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        auto entry = config().blackboard->getAnyLocked("cca_ros_context");
        if (entry && !entry->empty())
        {
            cca_ros_context_ = config().blackboard->get<std::shared_ptr<cca_ros::CcaRosContext>>("cca_ros_context");
        }
        else
        {
            cca_ros_context_ = std::make_shared<cca_ros::CcaRosContext>(node_);
            config().blackboard->set("cca_ros_context", cca_ros_context_);
        }
    }

    stop_source_ = std::stop_source{};

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
    const auto approach_reqs = std::vector<cca_ros::PlanningRequest>{*wbc_req, *arm_req};

    result_future_ = std::async(std::launch::async, [approach_reqs, grab_req, poses, this]() {
        return cca_ros_features::get_affordative_grasp_pose(
            cca_ros_context_, approach_reqs, *grab_req, *poses, timeout_, stop_source_.get_token());
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
    stop_source_.request_stop();
    if (result_future_.valid())
        result_future_.wait();
}

} // namespace cca_ros_behavior_features
