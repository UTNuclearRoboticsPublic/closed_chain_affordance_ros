#include <plan_req_builder/plan_req_builder.hpp>
#include <Eigen/Dense>
#include<sstream>

CcaPanel::CcaPanel(const std::string& node_name, const rclcpp::NodeOptions& node_options)
  : cca_ros::CcaRos(node_name, node_options)
{
  // Initialize subscribers
  button_press_subscriber_ = this->create_subscription<interactive_goal_interfaces::msg::ButtonPress>(
      "/button_press", 10, std::bind(&CcaPanel::buttonPressCallback, this, std::placeholders::_1));

  screw_info_subscriber_ = this->create_subscription<interactive_goal_interfaces::msg::ScrewInfo>(
      "/screw_info", 10, std::bind(&CcaPanel::screwInfoCallback, this, std::placeholders::_1));

  settings_subscriber_ = this->create_subscription<interactive_goal_interfaces::msg::AdvancedSettings>(
      "/settings", 10, std::bind(&CcaPanel::settingsCallback, this, std::placeholders::_1));

  // Subscribe to interactive marker feedback
  interactive_marker_feedback_subscriber_ =
      this->create_subscription<visualization_msgs::msg::InteractiveMarkerFeedback>(
          "/interactive_goals/feedback", 10,
          std::bind(&CcaPanel::interactiveMarkerFeedbackCallback, this, std::placeholders::_1));

  // Initialize publisher
  process_success_publisher_ =
      this->create_publisher<interactive_goal_interfaces::msg::ButtonPress>("process_success", 10);

}

void CcaPanel::buttonPressCallback(const interactive_goal_interfaces::msg::ButtonPress::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Button Callback");

  if (msg->plan)  // Check if plan has been triggered
  {
    // Combine with current_screw_info_ and current_arrow_pose_
    cca_ros::PlanningRequest planning_request = preparePlanningRequest(msg->visualize, msg->execute);

    const Eigen::VectorXd READY_CONFIG =
        (Eigen::VectorXd(6) << -0.00015592575073242188, -0.8980185389518738, 1.8094338178634644, 0.000377655029296875,
         -0.8991076946258545, 0.0015475749969482422)
            .finished();
    planning_request.start_state.robot = READY_CONFIG;

    if (run(planning_request))
    {
      RCLCPP_INFO(this->get_logger(), "Successfully called CCA action");
      block_until_trajectory_execution();  // Wait for execution to finish
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "CCA action failed");
    }
  }
  if (msg->stop)  // Check stop button
  {
    this->cancel_execution();
  }
}

void CcaPanel::screwInfoCallback(const interactive_goal_interfaces::msg::ScrewInfo::SharedPtr msg)
{
  current_screw_info_ = *msg;  // Update current screw info
   RCLCPP_INFO(this->get_logger(), "Received Screw Info:\n Category: %d\n Type: %d (0 = translation, 1 = rotation, 2 = screw)\n Location Frame: %s\n Pitch: %f\n Goal: %f",
              msg->category, msg->type, msg->location_frame.c_str(), msg->pitch, msg->goal);
}

void CcaPanel::settingsCallback(const interactive_goal_interfaces::msg::AdvancedSettings::SharedPtr msg)
{
  if (msg->change_accuracy)
  {
    config.accuracy = msg->accuracy;
  }
  if (msg->change_close_ang)
  {
    config.closure_err_threshold_ang = msg->closure_err_threshold_ang;
  }
  if (msg->change_close_lin)
  {
    config.closure_err_threshold_lin = msg->closure_err_threshold_lin;
  }
  if (msg->change_ik)
  {
    config.ik_max_itr = msg->ik_max_itr;
  }
  if (msg->change_trajectory)
  {
    current_trajectory_density = msg->trajectory_density;
  }
  if (msg->change_accuracy)
  {
    config.accuracy = msg->accuracy;
  }
  if (msg->virtual_screw_order == 0)
  {
    current_screw_order = affordance_util::VirtualScrewOrder::XYZ;
  }
  else if (msg->virtual_screw_order == 1)
  {
    current_screw_order = affordance_util::VirtualScrewOrder::YZX;
  }
  else if (msg->virtual_screw_order == 2)
  {
    current_screw_order = affordance_util::VirtualScrewOrder::ZXY;
  }
  else if (msg->virtual_screw_order == 3)
  {
    current_screw_order = affordance_util::VirtualScrewOrder::XY;
  }
  else if (msg->virtual_screw_order == 4)
  {
    current_screw_order = affordance_util::VirtualScrewOrder::YZ;
  }
  else if (msg->virtual_screw_order == 5)
  {
    current_screw_order = affordance_util::VirtualScrewOrder::ZX;
  }
}

void CcaPanel::interactiveMarkerFeedbackCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::SharedPtr feedback)
{
    RCLCPP_WARN(this->get_logger(), "Interactive marker feedback callback triggered");
  if (feedback->marker_name == "arrow_marker")
  {
    current_arrow_pose_ = feedback->pose;
    RCLCPP_INFO(this->get_logger(), "Arrow marker position updated to: [%f, %f, %f]", current_arrow_pose_.position.x,
                current_arrow_pose_.position.y, current_arrow_pose_.position.z);
  }
  else if (feedback->marker_name == "approach_frame")
  {
    current_frame_pose_ = feedback->pose;
    RCLCPP_INFO(this->get_logger(), "Frame marker position updated to: [%f, %f, %f]", feedback->pose.position.x,
                feedback->pose.position.y, feedback->pose.position.z);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Unknown marker name: %s", feedback->marker_name.c_str());
  }
}

void CcaPanel::block_until_trajectory_execution()
{
  rclcpp::Rate loop_rate(4);
  auto start_time = std::chrono::steady_clock::now();

  // *motion_status_ != cca_ros::Status::SUCCEEDED

  while (true)
  {
    //*motion_status_ == cca_ros::Status::UNKNOWN

    if (true)
    {
      RCLCPP_ERROR(this->get_logger(), "Motion was interrupted mid-execution.");
      auto current_time = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() > 60)
      {
        RCLCPP_ERROR(this->get_logger(), "Timeout waiting for motion to complete.");
        return;
      }
    }
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Exiting due to ROS signal");
      return;
    }
    loop_rate.sleep();
  }
}

bool CcaPanel::run(const cca_ros::PlanningRequest& planning_request)
{
  auto motion_status_ = planning_request.status;

// Define an Eigen format for pretty printing
  Eigen::IOFormat clean_fmt(4, 0, ", ", "\n", "[", "]");

  // Convert Eigen vectors to string for logging
  std::stringstream axis_ss, location_ss;
  axis_ss << planning_request.task_description.affordance_info.axis.transpose().format(clean_fmt);
  location_ss << planning_request.task_description.affordance_info.location.transpose().format(clean_fmt);

  // Use ROS2 logging instead of std::cout
  RCLCPP_INFO(rclcpp::get_logger("plan_req_builder"), "Here is the screw axis: %s", axis_ss.str().c_str());
  RCLCPP_INFO(rclcpp::get_logger("plan_req_builder"), "Here is the screw location: %s", location_ss.str().c_str());
  RCLCPP_INFO(rclcpp::get_logger("plan_req_builder"), "Here is the affordance goal: %f", planning_request.task_description.goal.affordance);

  return this->plan_visualize_and_execute(planning_request);
}

cca_ros::PlanningRequest CcaPanel::preparePlanningRequest(const bool visualize, const bool execute)
{
  cca_ros::PlanningRequest req;

  req.visualize_trajectory = visualize;
  req.execute_trajectory = execute;

  req.task_description.trajectory_density = current_trajectory_density;
  req.task_description.vir_screw_order = current_screw_order;
   RCLCPP_ERROR(this->get_logger(), "Current Screw Info:\n Category: %d\n Type: %d (0 = translation, 1 = rotation, 2 = screw)\n Location Frame: %s\n Pitch: %f\n Goal: %f",
              current_screw_info_.category, current_screw_info_.type, current_screw_info_.location_frame.c_str(), current_screw_info_.pitch, current_screw_info_.goal);

  if (current_screw_info_.category == 0)
  {
      RCLCPP_ERROR(this->get_logger(), "Cateory is affordance");
    req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);
  }
  else if (current_screw_info_.category == 1)
  {
      RCLCPP_ERROR(this->get_logger(), "Cateory is CGP");
    req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::CARTESIAN_GOAL);
  }
  else if (current_screw_info_.category == 2)
  {
      RCLCPP_ERROR(this->get_logger(), "Cateory is EE Orientation");
    req.task_description =
        cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);
  }
  else if (current_screw_info_.category == 3)
  {
      RCLCPP_ERROR(this->get_logger(), "Cateory is APPROACH");
    req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::APPROACH);
  }

  // Set affordance info types and pitch for screw in affordance and approach motion
  if (current_screw_info_.category == 0 || current_screw_info_.category == 3)
  {
      RCLCPP_ERROR(this->get_logger(), "Setting affordance screw types and pitch for affordance and APPROACH types");
    if (current_screw_info_.type == 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Screw is translation");
      req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
    }
    else if (current_screw_info_.type == 1)
    {
      RCLCPP_ERROR(this->get_logger(), "Screw is rotation");
      req.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
    }
    else if (current_screw_info_.type == 2)
    {
      RCLCPP_ERROR(this->get_logger(), "Screw is screw");
      req.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;
      req.task_description.affordance_info.pitch = current_screw_info_.pitch;
    }
  }

      RCLCPP_ERROR(this->get_logger(), "Setting affordance axes and goals");
  // Set goals and axis/location(not for in-place) for everything except for cgp
  if (current_screw_info_.category != 1)
  {
      RCLCPP_ERROR(this->get_logger(), "Category is not CGP. Calculating axis");
    RCLCPP_INFO(this->get_logger(), "Current arrow marker position updated to: [%f, %f, %f]", current_arrow_pose_.position.x,
                current_arrow_pose_.position.y, current_arrow_pose_.position.z);
    req.task_description.goal.affordance = current_screw_info_.goal;
    Eigen::Quaterniond q(current_arrow_pose_.orientation.w, 
    		     current_arrow_pose_.orientation.x,
    		     current_arrow_pose_.orientation.y,
    		     current_arrow_pose_.orientation.z);
    
    // Normalize to avoid numerical errors
    q.normalize();
    
    // Original direction of the arrow (aligned with X-axis)
    Eigen::Vector3d original_dir(1.0, 0.0, 0.0);
    
    // Apply quaternion rotation to get the new direction
    Eigen::Vector3d new_axis = q * original_dir;
    
    req.task_description.affordance_info.axis = new_axis;

    if (current_screw_info_.category != 2)
    {
      RCLCPP_ERROR(this->get_logger(), "Category is not CGP and EE orientation. Setting location");
      // Set location
      req.task_description.affordance_info.location = Eigen::Vector3d(
          current_arrow_pose_.position.x, current_arrow_pose_.position.y, current_arrow_pose_.position.z);
    }
  }

  // Set HTM for CGP and Approach motion task
  if (current_screw_info_.category == 1 || current_screw_info_.category == 3)
  {
      RCLCPP_ERROR(this->get_logger(), "Category is CGP or APPROACH. Looking up frame info.");
    Eigen::Isometry3d transform;
    tf2::convert(current_arrow_pose_, transform);
    req.task_description.goal.grasp_pose = transform.matrix();
  }
  RCLCPP_INFO(this->get_logger(), "Planning Request Built");

  return req;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<CcaPanel>("cca_ros", node_options);

  RCLCPP_INFO(node->get_logger(), "CCA Planner is active");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
