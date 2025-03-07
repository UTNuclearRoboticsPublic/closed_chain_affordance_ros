#include "cca_ros_util/cca_ros_util.hpp"

namespace cca_ros_util{

// Converts a cca_ros_viz_msgs::msg::PlanningRequest to a cca_ros PlanningRequest
cca_ros::PlanningRequest convert_cca_ros_action_to_req(const cca_ros_viz_msgs::msg::PlanningRequest& msg) { 
    cca_ros::PlanningRequest req;

    // Convert Planner Config
    req.planner_config.accuracy = msg.planner_config.accuracy;
    req.planner_config.closure_err_threshold_ang = msg.planner_config.closure_err_threshold_ang;
    req.planner_config.closure_err_threshold_lin = msg.planner_config.closure_err_threshold_lin;
    req.planner_config.ik_max_itr = msg.planner_config.ik_max_itr;
    req.planner_config.update_method = convert_ros_update_method_enum_to_cca(msg.planner_config.update_method.value);


    // Convert Task Description
    req.task_description.affordance_info.type = convert_ros_screw_type_enum_to_affordance_util(msg.task_description.affordance_info.type.value);


    // Convert axis, location, screw vector to Eigen::VectorXd
    req.task_description.affordance_info.axis = Eigen::VectorXd::Map(msg.task_description.affordance_info.axis.data(), msg.task_description.affordance_info.axis.size());
    req.task_description.affordance_info.location = Eigen::VectorXd::Map(msg.task_description.affordance_info.location.data(), msg.task_description.affordance_info.location.size());
    req.task_description.affordance_info.screw = Eigen::VectorXd::Map(msg.task_description.affordance_info.screw.data(), msg.task_description.affordance_info.screw.size());

    // Copy other fields
    req.task_description.affordance_info.location_frame = msg.task_description.affordance_info.location_frame;
    req.task_description.affordance_info.pitch = msg.task_description.affordance_info.pitch;

    // Convert location method enum
    req.task_description.affordance_info.location_method = convert_ros_screw_location_method_enum_to_affordance_util(msg.task_description.affordance_info.location_method.value);

    // Copy additional task description fields
    req.task_description.goal.affordance = msg.task_description.goal.affordance;
    req.task_description.goal.ee_orientation = Eigen::VectorXd::Map(msg.task_description.goal.ee_orientation.data(), 3);
    req.task_description.goal.grasp_pose = Eigen::MatrixXd::Map(msg.task_description.goal.grasp_pose.data(), 4, 4);  // Assuming 4x4 grasp pose
    req.task_description.goal.gripper = msg.task_description.goal.gripper;

    req.task_description.trajectory_density = msg.task_description.trajectory_density;

    // Convert motion_type and virtual screw order enums
    req.task_description.motion_type = convert_ros_motion_type_enum_to_cca(msg.task_description.motion_type.value);
    req.task_description.vir_screw_order = convert_ros_virtual_screw_order_enum_to_affordance_util(msg.task_description.vir_screw_order.value);

    // Convert gripper_goal_type enum
    req.task_description.gripper_goal_type = convert_ros_gripper_goal_type_enum_to_affordance_util(msg.task_description.gripper_goal_type.value);

    // Convert KinematicState (robot and gripper)
    req.start_state.robot = Eigen::VectorXd::Map(msg.start_state.robot.data(), msg.start_state.robot.size());
    req.start_state.gripper = msg.start_state.gripper;

    // Copy visualize_trajectory and execute_trajectory (booleans)
    req.visualize_trajectory = msg.visualize_trajectory;
    req.execute_trajectory = msg.execute_trajectory;

    return req;
}

// Converts a cca_ros PlanningRequest to a cca_ros_viz_msgs::msg::PlanningRequest
cca_ros_viz_msgs::msg::PlanningRequest convert_req_to_cca_ros_action(const cca_ros::PlanningRequest& req) {
    cca_ros_viz_msgs::msg::PlanningRequest msg;

    // Convert Planner Config
    msg.planner_config.accuracy = req.planner_config.accuracy;
    msg.planner_config.closure_err_threshold_ang = req.planner_config.closure_err_threshold_ang;
    msg.planner_config.closure_err_threshold_lin = req.planner_config.closure_err_threshold_lin;
    msg.planner_config.ik_max_itr = req.planner_config.ik_max_itr;
    msg.planner_config.update_method.value = convert_cca_update_method_to_ros(req.planner_config.update_method);


    // Convert Task Description
    msg.task_description.affordance_info.type.value = convert_affordance_util_screw_type_to_ros(req.task_description.affordance_info.type);


    // Convert axis, location, screw vector to std::vector
    std::copy(req.task_description.affordance_info.axis.data(),
              req.task_description.affordance_info.axis.data() + 3,
              msg.task_description.affordance_info.axis.begin());

    std::copy(req.task_description.affordance_info.location.data(),
              req.task_description.affordance_info.location.data() + 3,
              msg.task_description.affordance_info.location.begin());

    // You can directly assign the data from the Eigen vector to msg task description screw
    std::copy(req.task_description.affordance_info.screw.data(),
          req.task_description.affordance_info.screw.data() + msg.task_description.affordance_info.screw.size(),
          msg.task_description.affordance_info.screw.begin());

    // Copy other fields
    msg.task_description.affordance_info.location_frame = req.task_description.affordance_info.location_frame;
    msg.task_description.affordance_info.pitch = req.task_description.affordance_info.pitch;

    // Convert location method enum
    msg.task_description.affordance_info.location_method.value = convert_affordance_util_screw_location_method_to_ros(req.task_description.affordance_info.location_method);


    // Copy additional task description fields
    msg.task_description.goal.affordance = req.task_description.goal.affordance;
    std::copy(req.task_description.goal.ee_orientation.data(),
              req.task_description.goal.ee_orientation.data() + req.task_description.goal.ee_orientation.size(),
              msg.task_description.goal.ee_orientation.begin());

    std::copy(req.task_description.goal.grasp_pose.data(),
              req.task_description.goal.grasp_pose.data() + 16, // Assuming 4x4 matrix is flattened to 16 elements
              msg.task_description.goal.grasp_pose.begin());

    msg.task_description.goal.gripper = req.task_description.goal.gripper;
    msg.task_description.trajectory_density = req.task_description.trajectory_density;

    // Convert motion_type and virtual screw order enums
    msg.task_description.motion_type.value = convert_cca_motion_type_to_ros(req.task_description.motion_type);
    msg.task_description.vir_screw_order.value = convert_affordance_util_virtual_screw_order_to_ros(req.task_description.vir_screw_order);


    // Convert gripper_goal_type enum
    msg.task_description.gripper_goal_type.value = convert_affordance_util_gripper_goal_type_to_ros(req.task_description.gripper_goal_type);

    // Convert KinematicState (robot and gripper)
    msg.start_state.robot = std::vector<double>(req.start_state.robot.data(),
                                            req.start_state.robot.data() + req.start_state.robot.size());
    msg.start_state.gripper = req.start_state.gripper;

    // Copy visualize_trajectory and execute_trajectory (booleans)
    msg.visualize_trajectory = req.visualize_trajectory;
    msg.execute_trajectory = req.execute_trajectory;

    return msg;
}


cc_affordance_planner::UpdateMethod convert_ros_update_method_enum_to_cca(uint8_t update_method) {
        switch(update_method) {
            case 1:
                return cc_affordance_planner::UpdateMethod::INVERSE;
            case 2:
                return cc_affordance_planner::UpdateMethod::TRANSPOSE;
            case 0:
                return cc_affordance_planner::UpdateMethod::BEST;
            default:
                throw std::invalid_argument("Invalid CCA enum value for conversion.");
        }
    }

cc_affordance_planner::MotionType convert_ros_motion_type_enum_to_cca(uint8_t motion_type) {
        switch(motion_type) {
            case 1:
                return cc_affordance_planner::MotionType::APPROACH;
            case 0:
                return cc_affordance_planner::MotionType::AFFORDANCE;
            default:
                throw std::invalid_argument("Invalid CCA enum value for conversion.");
        }
    }


affordance_util::ScrewLocationMethod convert_ros_screw_location_method_enum_to_affordance_util(uint8_t location_method) {
        switch(location_method) {
            case 1:
                return affordance_util::ScrewLocationMethod::FROM_FK;
            case 2:
                return affordance_util::ScrewLocationMethod::FROM_FRAME_NAME;
            case 0:
                return affordance_util::ScrewLocationMethod::PROVIDED;
            default:
                throw std::invalid_argument("Invalid CCA enum value for conversion.");
        }
    }
affordance_util::GripperGoalType convert_ros_gripper_goal_type_enum_to_affordance_util(uint8_t gripper_goal_type) {
    switch(gripper_goal_type) {
        case 1:
            return affordance_util::GripperGoalType::CONTINUOUS;
        case 0:
            return affordance_util::GripperGoalType::CONSTANT;
        default:
            throw std::invalid_argument("Invalid AffordanceUtil enum value for conversion.");
    }
}

affordance_util::ScrewType convert_ros_screw_type_enum_to_affordance_util(uint8_t screw_type) {
    switch(screw_type) {
        case 1:
            return affordance_util::ScrewType::ROTATION;
        case 2:
            return affordance_util::ScrewType::TRANSLATION;
        case 3:
            return affordance_util::ScrewType::SCREW;
        case 0:
            return affordance_util::ScrewType::UNSET;
        default:
            throw std::invalid_argument("Invalid AffordanceUtil enum value for conversion.");
    }
}
affordance_util::VirtualScrewOrder convert_ros_virtual_screw_order_enum_to_affordance_util(uint8_t virtual_screw_order) {
    switch(virtual_screw_order) {
        case 1:
            return affordance_util::VirtualScrewOrder::XYZ;
        case 2:
            return affordance_util::VirtualScrewOrder::YZX;
        case 3:
            return affordance_util::VirtualScrewOrder::ZXY;
        case 4:
            return affordance_util::VirtualScrewOrder::XY;
        case 5:
            return affordance_util::VirtualScrewOrder::YZ;
        case 6:
            return affordance_util::VirtualScrewOrder::ZX;
        case 0:
            return affordance_util::VirtualScrewOrder::NONE;
        default:
            throw std::invalid_argument("Invalid AffordanceUtil enum value for conversion.");
    }
}

uint8_t convert_cca_update_method_to_ros(cc_affordance_planner::UpdateMethod update_method) {
    switch(update_method) {
        case cc_affordance_planner::UpdateMethod::INVERSE:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::UpdateMethod::INVERSE);
        case cc_affordance_planner::UpdateMethod::TRANSPOSE:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::UpdateMethod::TRANSPOSE);
        case cc_affordance_planner::UpdateMethod::BEST:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::UpdateMethod::BEST);
        default:
            throw std::invalid_argument("Invalid CCA UpdateMethod enum value for conversion.");
    }
}

uint8_t convert_cca_motion_type_to_ros(cc_affordance_planner::MotionType motion_type) {
    switch(motion_type) {
        case cc_affordance_planner::MotionType::APPROACH:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::MotionType::APPROACH);
        case cc_affordance_planner::MotionType::AFFORDANCE:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::MotionType::AFFORDANCE);
        default:
            throw std::invalid_argument("Invalid CCA MotionType enum value for conversion.");
    }
}

uint8_t convert_affordance_util_screw_location_method_to_ros(affordance_util::ScrewLocationMethod location_method) {
    switch(location_method) {
        case affordance_util::ScrewLocationMethod::FROM_FK:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::ScrewLocationMethod::FROM_FK);
        case affordance_util::ScrewLocationMethod::FROM_FRAME_NAME:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::ScrewLocationMethod::FROM_FRAME_NAME);
        case affordance_util::ScrewLocationMethod::PROVIDED:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::ScrewLocationMethod::PROVIDED);
        default:
            throw std::invalid_argument("Invalid AffordanceUtil ScrewLocationMethod enum value for conversion.");
    }
}

uint8_t convert_affordance_util_gripper_goal_type_to_ros(affordance_util::GripperGoalType gripper_goal_type) {
    switch(gripper_goal_type) {
        case affordance_util::GripperGoalType::CONTINUOUS:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::GripperGoalType::CONTINUOUS);
        case affordance_util::GripperGoalType::CONSTANT:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::GripperGoalType::CONSTANT);
        default:
            throw std::invalid_argument("Invalid AffordanceUtil GripperGoalType enum value for conversion.");
    }
}

uint8_t convert_affordance_util_screw_type_to_ros(affordance_util::ScrewType screw_type) {
    switch(screw_type) {
        case affordance_util::ScrewType::ROTATION:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::ScrewType::ROTATION);
        case affordance_util::ScrewType::TRANSLATION:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::ScrewType::TRANSLATION);
        case affordance_util::ScrewType::SCREW:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::ScrewType::SCREW);
        case affordance_util::ScrewType::UNSET:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::ScrewType::UNSET);
        default:
            throw std::invalid_argument("Invalid AffordanceUtil ScrewType enum value for conversion.");
    }
}

uint8_t convert_affordance_util_virtual_screw_order_to_ros(affordance_util::VirtualScrewOrder virtual_screw_order) {
    switch(virtual_screw_order) {
        case affordance_util::VirtualScrewOrder::XYZ:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::VirtualScrewOrder::XYZ);
        case affordance_util::VirtualScrewOrder::YZX:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::VirtualScrewOrder::YZX);
        case affordance_util::VirtualScrewOrder::ZXY:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::VirtualScrewOrder::ZXY);
        case affordance_util::VirtualScrewOrder::XY:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::VirtualScrewOrder::XY);
        case affordance_util::VirtualScrewOrder::YZ:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::VirtualScrewOrder::YZ);
        case affordance_util::VirtualScrewOrder::ZX:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::VirtualScrewOrder::ZX);
        case affordance_util::VirtualScrewOrder::NONE:
            return static_cast<uint8_t>(cca_ros_viz_msgs::msg::VirtualScrewOrder::NONE);
        default:
            throw std::invalid_argument("Invalid AffordanceUtil VirtualScrewOrder enum value for conversion.");
    }
}

}
