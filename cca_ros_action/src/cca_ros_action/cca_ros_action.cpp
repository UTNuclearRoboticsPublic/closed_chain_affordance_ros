#include "cca_ros_action/cca_ros_action.hpp"
namespace cca_ros_action
{
	CcaRosActionServer::CcaRosActionServer(const std::string& node_name, const rclcpp::NodeOptions & node_options)
    : cca_ros::CcaRos(node_name, node_options),
      as_server_name_("/cca_ros_action"), timeout_secs_(60)
    {
        // Initialize the action server
        action_server_ = rclcpp_action::create_server<CcaRosAction>(
            this,
            as_server_name_,
            std::bind(&CcaRosActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CcaRosActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&CcaRosActionServer::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Action server '%s' initialized.", as_server_name_.c_str());
    }

    rclcpp_action::GoalResponse CcaRosActionServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const CcaRosAction::Goal> goal)
    {
	RCLCPP_INFO(this->get_logger(), "Received a new goal request on action server '%s' with affordance goal: %.4f.", as_server_name_.c_str(), goal->req.task_description.goal.affordance);
        (void)uuid;

        // Validate the goal (you can add additional checks)
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse CcaRosActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleCcaRosAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request on action server '%s'.", as_server_name_.c_str());
        (void)goal_handle;

	this->cancel_execution();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void CcaRosActionServer::handle_accepted(const std::shared_ptr<GoalHandleCcaRosAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Accepted goal on action server '%s'.", as_server_name_.c_str());
        // Process the goal in a separate thread for execution
        std::thread{std::bind(&CcaRosActionServer::execute_action, this, std::placeholders::_1), goal_handle}.detach();
    }

    void CcaRosActionServer::execute_action(const std::shared_ptr<GoalHandleCcaRosAction> goal_handle)
    {
        const auto goal = goal_handle->get_goal();

        RCLCPP_INFO(this->get_logger(), "Executing action on server '%s'.", as_server_name_.c_str());

	// Convert the ROS message to cca_ros planning request struct
	cca_ros::PlanningRequest req = cca_ros_util::convert_cca_ros_action_to_req(goal->req); 

        RCLCPP_INFO_STREAM(this->get_logger(), "Here is the converted planning request: ");
        RCLCPP_INFO_STREAM(this->get_logger(), "Affordance axis: "<<req.task_description.affordance_info.axis.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Affordance location: "<<req.task_description.affordance_info.location.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Affordance goal: "<<req.task_description.goal.affordance);
        RCLCPP_INFO_STREAM(this->get_logger(), "Start state: "<<req.start_state.robot.transpose());
        RCLCPP_INFO_STREAM(this->get_logger(), "Trajectory density: "<<req.task_description.trajectory_density);
        RCLCPP_INFO_STREAM(this->get_logger(), "Accuracy: "<<req.planner_config.accuracy);
	// req.task_description.goal.ee_orientation.conservativeResize(0); 
	
        RCLCPP_INFO_STREAM(this->get_logger(), "EE orientation: "<<req.task_description.goal.ee_orientation.transpose());
	std::string type;
	if (req.task_description.affordance_info.type == affordance_util::ScrewType::TRANSLATION) {
	    type = "translation";
	} else if (req.task_description.affordance_info.type == affordance_util::ScrewType::ROTATION) {
	    type = "rotation";
	} else if (req.task_description.affordance_info.type == affordance_util::ScrewType::SCREW) {
	    type = "screw";
	} else {
	    type = "unset";
	}

	std::string motion_type;
	if (req.task_description.motion_type == cc_affordance_planner::MotionType::AFFORDANCE) {
	    motion_type = "affordance";
	} else {
	    motion_type = "approach";
	}
        RCLCPP_INFO_STREAM(this->get_logger(), "type: "<<type);
        RCLCPP_INFO_STREAM(this->get_logger(), "motion type: "<<motion_type);
	std::string virtual_screw_order;
switch (req.task_description.vir_screw_order) {
    case affordance_util::VirtualScrewOrder::XYZ:
        virtual_screw_order = "XYZ";
        break;
    case affordance_util::VirtualScrewOrder::YZX:
        virtual_screw_order = "YZX";
        break;
    case affordance_util::VirtualScrewOrder::ZXY:
        virtual_screw_order = "ZXY";
        break;
    case affordance_util::VirtualScrewOrder::XY:
        virtual_screw_order = "XY";
        break;
    case affordance_util::VirtualScrewOrder::YZ:
        virtual_screw_order = "YZ";
        break;
    case affordance_util::VirtualScrewOrder::ZX:
        virtual_screw_order = "ZX";
        break;
    case affordance_util::VirtualScrewOrder::NONE:
        virtual_screw_order = "NONE";
        break;
    default:
        virtual_screw_order = "UNKNOWN";
        break;
}
        RCLCPP_INFO_STREAM(this->get_logger(), "Virtual screw oder: "<<virtual_screw_order);


	// cca_ros::PlanningRequest req;
	// const Eigen::VectorXd READY_CONFIG =
        // (Eigen::VectorXd(6) << -0.00015592575073242188, -0.8980185389518738, 1.8094338178634644, 0.000377655029296875,
         // -0.8991076946258545, 0.0015475749969482422)
            // .finished();
    // req.start_state.robot = READY_CONFIG;
	// req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

        // // Affordance info
        // req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
        // req.task_description.affordance_info.axis = Eigen::Vector3d(0, 0, 1);
        // req.task_description.affordance_info.location = Eigen::Vector3d::Zero();

        // // Goals
        // req.task_description.goal.affordance = 0.5;
        // req.task_description.goal.ee_orientation =
        //     (Eigen::VectorXd(1) << M_PI / 12.0).finished(); // Optionally, specify EE orientation constraint as rpy. In
        //                                                     // this example we are only constraining yaw.
			

        if (!this->plan_visualize_and_execute(req)) 
        {
            RCLCPP_ERROR(this->get_logger(), "Execution failed on action server '%s'.", as_server_name_.c_str());
            goal_handle->abort(std::make_shared<CcaRosAction::Result>());
            return;
        }

        start_time_ = std::chrono::steady_clock::now();

        while (rclcpp::ok())
        {
            auto current_time = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count() > timeout_secs_)
            {
                RCLCPP_ERROR(this->get_logger(), "Timed out waiting for action request to complete on server '%s'.", as_server_name_.c_str());
                goal_handle->abort(std::make_shared<CcaRosAction::Result>());
                return;
            }

            if (*req.status == cca_ros::Status::SUCCEEDED)
            {
                RCLCPP_INFO(this->get_logger(), "Action successfully completed on server '%s'.", as_server_name_.c_str());
                goal_handle->succeed(std::make_shared<CcaRosAction::Result>());
                return;
            }
            else if (*req.status == cca_ros::Status::UNKNOWN)
            {
                RCLCPP_ERROR(this->get_logger(), "Action was interrupted mid-execution on server '%s'.", as_server_name_.c_str());
                goal_handle->abort(std::make_shared<CcaRosAction::Result>());
                return;
            }

	    std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    }
} // namespace cca_ros_action

