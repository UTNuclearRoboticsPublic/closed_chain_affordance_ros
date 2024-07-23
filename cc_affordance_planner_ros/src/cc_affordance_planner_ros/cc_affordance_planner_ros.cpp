#include <cc_affordance_planner_ros/cc_affordance_planner_ros.hpp>

CcAffordancePlannerRos::CcAffordancePlannerRos(const std::string &node_name, const rclcpp::NodeOptions &node_options)
    : Node(node_name, node_options), // Use new class name
      node_logger_(this->get_logger()),
      plan_and_viz_ss_name_("/moveit_plan_and_viz_server")
{
    // Extract CC Affordance parameters regarding ROS-related setup
    traj_execution_as_name_ = this->get_parameter("cca_robot_as").as_string();
    robot_description_parameter_ = this->get_parameter("cca_robot_description_parameter").as_string();
    planning_group_ = this->get_parameter("cca_planning_group").as_string();
    rviz_fixed_frame_ = this->get_parameter("rviz_fixed_frame").as_string();
    const std::string joint_states_topic = this->get_parameter("cca_joint_states_topic").as_string();
    const std::string robot_name = this->get_parameter("cca_robot").as_string();

    // Extract robot config info
    const std::string robot_config_file_path = CcAffordancePlannerRos::get_cc_affordance_robot_description_(robot_name);
    const affordance_util::RobotConfig &robotConfig = affordance_util::robot_builder(robot_config_file_path);
    robot_slist_ = robotConfig.Slist;
    M_ = robotConfig.M;
    ref_frame_ = robotConfig.ref_frame_name;
    tool_frame_ = robotConfig.tool_name;
    joint_names_ = robotConfig.joint_names;

    // Initialize clients and subscribers
    traj_execution_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, traj_execution_as_name_);
    plan_and_viz_client_ = this->create_client<MoveItPlanAndViz>(plan_and_viz_ss_name_);
    gripper_open_client_ = this->create_client<std_srvs::srv::Trigger>("/spot_manipulation_driver/open_gripper");
    gripper_close_client_ = this->create_client<std_srvs::srv::Trigger>("/spot_manipulation_driver/close_gripper");
    mini_unstow_client_ = this->create_client<std_srvs::srv::Trigger>("/spot_manipulation_driver/mini_unstow_arm");
    navigation_action_client_ =
        rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/navigate_to_pose");
    /* rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "/spot_driver/navigate_to"); */
    joint_states_sub_ = this->create_subscription<JointState>(
        joint_states_topic, 1000, std::bind(&CcAffordancePlannerRos::joint_states_cb_, this, std::placeholders::_1));

    // Construct buffer to lookup affordance location from apriltag using tf data
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

void CcAffordancePlannerRos::navigate_to_pose(const nav2_msgs::action::NavigateToPose::Goal &navigation_goal)
{
    auto goal_handle_future = navigation_action_client_->async_send_goal(navigation_goal);
    /* auto result = rclcpp::spin_until_future_complete(this->get_node_base_interface(), goal_handle_future, */
    /*                                                  std::chrono::milliseconds(5)); */
    /* rclcpp::sleep_for(std::chrono::seconds(5)); */
    RCLCPP_INFO(node_logger_, "Navigation action server call concluded");
}
bool CcAffordancePlannerRos::run_cc_affordance_planner_approach_motion(
    const cc_affordance_planner::PlannerConfig &plannerConfig, affordance_util::ScrewInfo &aff,
    const Eigen::VectorXd &sec_goal, Eigen::VectorXd grasp_config, const size_t &gripper_control_par,
    const std::string &vir_screw_order, Eigen::VectorXd robot_start_config)
{
    Eigen::Matrix4d grasp_pose;
    nav2_msgs::action::NavigateToPose::Goal navigation_goal;
    navigation_goal.pose.header.frame_id = "map";
    navigation_goal.pose.header.stamp = this->get_clock()->now();
    // If tag frame is specified then, we lookup affordance location from tag
    if (!aff.location_frame.empty())
    {

        RCLCPP_INFO_STREAM(node_logger_, "Ready to read affordance location from apriltag? y or Y for yes.");
        std::string conf;
        std::cin >> conf;
        if (conf != "y" && conf != "Y")
        {
            throw std::runtime_error("You indicated you are not ready to read affordance location");
        }
        /* const Eigen::Isometry3d aff_htm = affordance_util_ros::get_htm(ref_frame_, aff.location_frame, *tf_buffer_);
         */
        /* /1* aff.location = aff_htm.translation(); *1/ */
        /* aff.location = Eigen::Vector3d(0.0, 0.0, 0.0); // moving_a_stool */
        /* Eigen::Matrix4d offset = Eigen::Matrix4d::Identity(); */
        /* Eigen::Matrix4d aff_htm_m = aff_htm.matrix(); */
        /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.45, 0.27); */
        /* grasp_pose = aff_htm_m * offset; */
        /* grasp_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); */
        Eigen::Matrix4d body_offset = Eigen::Matrix4d::Identity();
        /* body_offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 1.4); */
        body_offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 1.2);
        /* body_offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, 0.0, 0.8); */
        const Eigen::Isometry3d aff_htm_map = affordance_util_ros::get_htm("map", aff.location_frame, *tf_buffer_);
        /* const Eigen::Isometry3d body_or = affordance_util_ros::get_htm("map", "arm0_base_link", *tf_buffer_); */
        Eigen::Matrix4d tag_to_body_or = Eigen::Matrix4d::Identity();
        tag_to_body_or.block<3, 1>(0, 0) = Eigen::Vector3d(0, 0, -1);
        tag_to_body_or.block<3, 1>(0, 1) = Eigen::Vector3d(-1, 0, 0);
        tag_to_body_or.block<3, 1>(0, 2) = Eigen::Vector3d(0, 1, 0);
        const Eigen::Matrix4d body_or = aff_htm_map.matrix() * tag_to_body_or;
        /* const Eigen::Quaterniond body_or_quat(body_or.rotation()); */
        const Eigen::Quaterniond body_or_quat(body_or.block<3, 3>(0, 0));
        Eigen::Matrix4d aff_htm_map_m = aff_htm_map.matrix() * body_offset;
        navigation_goal.pose.pose.position.x = aff_htm_map_m(0, 3);
        navigation_goal.pose.pose.position.y = aff_htm_map_m(1, 3);
        /* navigation_goal.pose.pose.orientation.x = 0; */
        /* navigation_goal.pose.pose.orientation.y = 0; */
        /* navigation_goal.pose.pose.orientation.z = 0; */
        /* navigation_goal.pose.pose.orientation.w = 1; */
        navigation_goal.pose.pose.orientation.x = body_or_quat.x();
        navigation_goal.pose.pose.orientation.y = body_or_quat.y();
        navigation_goal.pose.pose.orientation.z = body_or_quat.z();
        navigation_goal.pose.pose.orientation.w = body_or_quat.w();

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = navigation_goal.pose.header.frame_id;
        t.child_frame_id = "carrot";
        t.transform.translation.x = navigation_goal.pose.pose.position.x;
        t.transform.translation.y = navigation_goal.pose.pose.position.y;
        t.transform.rotation.x = navigation_goal.pose.pose.orientation.x;
        t.transform.rotation.y = navigation_goal.pose.pose.orientation.y;
        t.transform.rotation.z = navigation_goal.pose.pose.orientation.z;
        t.transform.rotation.w = navigation_goal.pose.pose.orientation.w;
        rclcpp::Rate loop_rate(4.0);

        for (int i = 0; i <= 20; i++)
        {
            tf_broadcaster_->sendTransform(t);
            loop_rate.sleep();
        }
        /* rclcpp::shutdown(); */
    }
    const Eigen::Matrix<double, 6, 1> aff_screw = affordance_util::get_screw(aff); // compute affordance screw

    RCLCPP_INFO_STREAM(node_logger_, "Navigating to affordance");
    navigate_to_pose(navigation_goal);
    std::cout << "Confirm navigation goal reached?" << std::endl;
    std::string nav_conf;
    std::cin >> nav_conf;

    const Eigen::Isometry3d aff_htm = affordance_util_ros::get_htm(ref_frame_, aff.location_frame, *tf_buffer_);
    /* aff.location = aff_htm.translation(); */
    aff.location = Eigen::Vector3d(0.0, 0.0, 0.0); // moving_a_stool
    Eigen::Matrix4d offset = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d aff_htm_m = aff_htm.matrix();
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.45, 0.27); */
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.27, 0.45); */
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -1.0, 0.45); */
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.35, 0.35); // working */
    offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.33, 0.28); //
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.57, 0.27); */
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.57, 0.45); */
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.0, -0.65, 0.6); */
    /* offset.block<3, 1>(0, 3) = Eigen::Vector3d(0.3, 0.0, 0.0); */
    std::cout << "Here is the offset:\n" << offset << std::endl;
    grasp_pose = aff_htm_m * offset;
    /* grasp_pose = aff_htm_m; */
    /* grasp_pose.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity(); */
    double theta = 0;
    // about x axis
    /* grasp_pose.block<3, 1>(0, 0) = Eigen::Vector3d(1, 0.0, 0.0); */
    /* grasp_pose.block<3, 1>(0, 1) = Eigen::Vector3d(0.0, cos(theta), sin(theta)); */
    /* grasp_pose.block<3, 1>(0, 2) = Eigen::Vector3d(0.0, -sin(theta), cos(theta)); */
    // about y axis
    grasp_pose.block<3, 1>(0, 0) = Eigen::Vector3d(cos(theta), 0.0, -sin(theta));
    grasp_pose.block<3, 1>(0, 1) = Eigen::Vector3d(0.0, 1, 0.0);
    grasp_pose.block<3, 1>(0, 2) = Eigen::Vector3d(sin(theta), 0.0, cos(theta));
    // about y axis
    /* grasp_pose.block<3, 1>(0, 0) = Eigen::Vector3d(cos(theta), sin(theta), 0.0); */
    /* grasp_pose.block<3, 1>(0, 1) = Eigen::Vector3d(-sin(theta), cos(theta), 0.0); */
    /* grasp_pose.block<3, 1>(0, 2) = Eigen::Vector3d(0.0, 0.0, 1.0); */

    const Eigen::Quaterniond grasp_or(grasp_pose.block<3, 3>(0, 0));
    // Print Grasp Pose

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = ref_frame_;
    t.child_frame_id = "carrot";
    t.transform.translation.x = grasp_pose(0, 3);
    t.transform.translation.y = grasp_pose(1, 3);
    t.transform.translation.z = grasp_pose(2, 3);
    t.transform.rotation.x = grasp_or.x();
    t.transform.rotation.y = grasp_or.y();
    t.transform.rotation.z = grasp_or.z();
    t.transform.rotation.w = grasp_or.w();
    rclcpp::Rate loop_rate(4.0);

    std::cout << "PRINTING GRASP POSE" << std::endl;
    for (int i = 0; i <= 20; i++)
    {
        tf_broadcaster_->sendTransform(t);
        loop_rate.sleep();
    }

    /***********************************************************************************/
    // open gripper
    rclcpp::sleep_for(std::chrono::seconds(2));
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    /* //  Wait for the service to be activated */
    /* while (!gripper_open_client_->wait_for_service(1s)) */
    /* { */
    /*     // If ROS is shutdown before the service is activated, show this error */
    /*     if (!rclcpp::ok()) */
    /*     { */
    /*         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting."); */
    /*         return 0; */
    /*     } */
    /*     // Print in the screen some information so the user knows what is happening */
    /*     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again..."); */
    /* } */
    // Client sends its asynchronous request
    auto mini_unstow_result = mini_unstow_client_->async_send_request(request);
    auto mini_unstow_response = mini_unstow_result.get(); // blocks until response is received
    // Wait for the result
    if (mini_unstow_response->success)
    {
        // Get the response's success field to see if all checks passed
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mini Unstow service called successfully");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call mini unstow service");
    }
    std::cout << "Confirm mini unstow" << std::endl;
    std::string mini_unstow_conf;
    std::cin >> mini_unstow_conf;

    // Get joint states at the start configuration of the affordance
    if (robot_start_config.size() == 0) // Non-zero if testing or planning without the joint_states topic
    {
        robot_start_config = get_aff_start_joint_states_();
    }

    // Compute approach screw
    /* Eigen::MatrixXd test_slist(6, 10); */
    const Eigen::MatrixXd cc_slist_a =
        affordance_util::compose_cc_model_slist(robot_slist_, robot_start_config, M_, aff_screw, vir_screw_order);
    /* test_slist << robot_slist_, cc_slist_a.col(6), cc_slist_a.col(7), cc_slist_a.col(8), aff_screw; */
    /* Eigen::VectorXd test_thetalist(10); */
    /* test_thetalist << robot_start_config, 0.0, 0.0, 0.0, 0.0; */
    /* const Eigen::Matrix4d start_pose = affordance_util::FKinSpace(M_, test_slist, test_thetalist); */
    const Eigen::Matrix4d start_pose = affordance_util::FKinSpace(M_, robot_slist_, robot_start_config);
    /* const Eigen::Matrix4d grasp_pose = affordance_util::FKinSpace(M_, robot_slist_, grasp_config); */
    /* Eigen::Matrix4d grasp_pose = affordance_util::FKinSpace(M_, robot_slist_, grasp_config); */
    /* grasp_pose.block<3, 1>(0, 3) = grasp_pose.block<3, 1>(0, 3) + Eigen::Vector3d(-0.2, 0, 0.0); */

    const Eigen::Matrix<double, 6, 1> approach_twist =
        affordance_util::Adjoint(start_pose) *
        affordance_util::se3ToVec(affordance_util::MatrixLog6(affordance_util::TransInv(start_pose) * grasp_pose));

    const Eigen::Matrix<double, 6, 1> approach_screw = approach_twist / approach_twist.norm();

    // Compose cc model and affordance goal
    /* const Eigen::MatrixXd start_jac = affordance_util::JacobianSpace(robot_slist_, robot_start_config); */
    /* Eigen::MatrixXd approach_cc_slist(cc_slist.rows(), cc_slist.cols() + 1); */
    /* approach_cc_slist << start_jac, cc_slist.col(6), cc_slist.col(7), cc_slist.col(8), cc_slist.col(9),
     * approach_screw; */
    /* const Eigen::MatrixXd cc_slist_a = */
    /* affordance_util::compose_cc_model_slist(robot_slist_, robot_start_config, M_, aff_screw, vir_screw_order); */
    /* cc_slist_a.col(cc_slist_a.cols() - 1) = -aff_screw; */
    /* Eigen::MatrixXd approach_cc_slist(cc_slist.rows(), cc_slist.cols() + 1); */
    /* Eigen::MatrixXd approach_cc_slist(cc_slist_a.rows(), cc_slist_a.cols() + 1); */
    /* approach_cc_slist << cc_slist_a, approach_screw; */
    Eigen::MatrixXd approach_cc_slist(cc_slist_a.rows(), 7);
    approach_cc_slist << cc_slist_a.block<6, 6>(0, 0), approach_screw;
    /* approach_cc_slist << cc_slist_a.block<6, 6>(0, 0), approach_screw, cc_slist_a.col(6), cc_slist_a.col(7), */
    /* cc_slist_a.col(8), cc_slist_a.col(9); */
    /* Eigen::MatrixXd approach_cc_slist(cc_slist_a.rows(), cc_slist_a.cols()); */
    /* approach_cc_slist << cc_slist_a; */
    /* approach_cc_slist.col(approach_cc_slist.cols() - 1) = approach_screw; */

    /* std::cout << "DEBUG FLAG" << std::endl; */
    /* const Eigen::Vector3d approach_vector = grasp_pose.block<3, 1>(0, 3) - start_pose.block<3, 1>(0, 3); */
    /* Eigen::Vector2d approach_goal; */
    /* Eigen::VectorXd approach_goal(5); */
    /* Eigen::VectorXd approach_goal(4); */
    /* Eigen::VectorXd approach_goal(3); */
    /* Eigen::VectorXd approach_goal(2); */
    Eigen::VectorXd approach_goal(1);
    /* approach_goal << 0.0, approach_vector.norm() + 0.6; */
    /* approach_goal << approach_twist.norm(), 0.0, 0.0, 0.0, 0.0; */
    /* approach_goal << 0.0, 0.0, 0.0, approach_twist.norm(); */
    /* approach_goal << 0.2, 0.0, approach_twist.norm(); */
    /* approach_goal << 0.0, approach_twist.norm(); */
    approach_goal << approach_twist.norm();
    std::cout << std::fixed << std::setprecision(3); // Display up to 4 decimal places
    /* std::cout << "Here is the closed-chain screw list:\n" << cc_slist << std::endl; */
    std::cout << "Here is the closed-chain screw list with approach screw:\n" << approach_cc_slist << std::endl;
    std::cout << "Here is the start pose:\n" << start_pose << std::endl;
    std::cout << "Here is the grasp pose:\n" << grasp_pose << std::endl;
    /* /1* std::cout << "Here is the approach vector:\n" << approach_vector << std::endl; *1/ */
    std::cout << "Here is the approach goal:\n" << approach_goal << std::endl;

    // Run the planner
    cc_affordance_planner::PlannerResult approachResult =
        /* cc_affordance_planner::generate_joint_trajectory(plannerConfig, approach_cc_slist, approach_goal, 5); */
        /* cc_affordance_planner::generate_joint_trajectory(plannerConfig, approach_cc_slist, approach_goal, 4); */
        /* cc_affordance_planner::generate_joint_trajectory(plannerConfig, approach_cc_slist, approach_goal, 3); */
        /* cc_affordance_planner::generate_joint_trajectory(plannerConfig, approach_cc_slist, approach_goal, 2); */
        cc_affordance_planner::generate_joint_trajectory(plannerConfig, approach_cc_slist, approach_goal, 1);
    /* cc_affordance_planner::PlannerResult plannerResult = */
    /*     cc_affordance_planner::generate_joint_trajectory(plannerConfig, cc_slist, sec_goal, gripper_control_par); */
    if (approachResult.success)
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with "
                                             << approachResult.traj_full_or_partial << " solution, planning took "
                                             << approachResult.planning_time.count() << " microseconds, and "
                                             << approachResult.update_method << " update method was used.");
    }
    else
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
    }

    // Print planner result
    std::vector<Eigen::VectorXd> solution = approachResult.joint_traj;
    for (auto &point : solution)
    {

        point.head(6) = point.head(6) + robot_start_config;
    }
    solution.insert(solution.begin(), robot_start_config);
    bool success = visualize_and_execute_trajectory_(solution, Eigen::VectorXd::Zero(6), aff.axis, aff.location);

    /* const Eigen::Matrix4d computed_grasp_pose = affordance_util::FKinSpace(M_, robot_slist_,
     * solution.back().head(6)); */
    /* std::cout << "Here is the computed grasp pose:\n" << computed_grasp_pose << std::endl; */
    /***********************************************************************************/
    // open gripper
    rclcpp::sleep_for(std::chrono::seconds(2));
    /* auto request = std::make_shared<std_srvs::srv::Trigger::Request>(); */
    /* //  Wait for the service to be activated */
    /* while (!gripper_open_client_->wait_for_service(1s)) */
    /* { */
    /*     // If ROS is shutdown before the service is activated, show this error */
    /*     if (!rclcpp::ok()) */
    /*     { */
    /*         RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting."); */
    /*         return 0; */
    /*     } */
    /*     // Print in the screen some information so the user knows what is happening */
    /*     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again..."); */
    /* } */
    // Client sends its asynchronous request
    auto open_result = gripper_open_client_->async_send_request(request);
    auto open_response = open_result.get(); // blocks until response is received
    // Wait for the result
    if (open_response->success)
    {
        // Get the response's success field to see if all checks passed
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper open service called successfully");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call Gripper open service");
    }
    std::cout << "Confirm Gripper open" << std::endl;
    std::string gripper_conf;
    std::cin >> gripper_conf;

    /***********************************************************************************/
    /* grasp_config = get_aff_start_joint_states_(); */
    /* std::cout << "Here is the grasp config:\n" << grasp_config << std::endl; */
    /* const Eigen::MatrixXd cc_slist = */
    /* affordance_util::compose_cc_model_slist(robot_slist_, grasp_config, M_, aff_screw, vir_screw_order); */
    /* std::vector<Eigen::VectorXd> affResult = run_cc_affordance_planner_affordance_motion( */
    /* plannerConfig, aff, sec_goal, gripper_control_par, vir_screw_order, grasp_config); */
    /* plannerConfig, aff, sec_goal, gripper_control_par, vir_screw_order); */
    /* for (auto &point : affResult) */
    /* { */

    /*     point.head(6) = point.head(6) + grasp_config; */
    /* } */
    /* affResult.insert(affResult.begin(), grasp_config); */
    /* std::vector<Eigen::VectorXd> start_point; */
    /* start_point.push_back(affResult[0]); */
    /* success = visualize_and_execute_trajectory_(start_point, Eigen::VectorXd::Zero(6), aff.axis, aff.location); */
    /* rclcpp::sleep_for(std::chrono::seconds(2)); */
    /***********************************************************************************/
    auto close_result = gripper_close_client_->async_send_request(request);
    auto close_response = close_result.get(); // blocks until response is received
    // Wait for the result
    if (close_response->success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Gripper close service called successfully");
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call Gripper close service");
    }
    std::cout << "Confirm Gripper close" << std::endl;
    std::cin >> gripper_conf;
    /***********************************************************************************/
    /* std::vector<Eigen::VectorXd> affResult = run_cc_affordance_planner_affordance_motion( */
    /* plannerConfig, aff, sec_goal, gripper_control_par, vir_screw_order, grasp_config); */
    std::vector<Eigen::VectorXd> affResult =
        run_cc_affordance_planner_affordance_motion(plannerConfig, aff, sec_goal, gripper_control_par, vir_screw_order);
    /* success = visualize_and_execute_trajectory_(affResult, Eigen::VectorXd::Zero(6), aff.axis, aff.location); */
    /* solution.insert(solution.end(), affResult.begin(), affResult.end()); */
    /* if (approachResult.success) */
    /* if (approachResult.success) */
    /* if (approachResult.success && plannerResult.success) */
    /* { */
    /* RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with " */
    /*                                      << approachResult.traj_full_or_partial << " solution, planning took " */
    /*                                      << approachResult.planning_time.count() << " microseconds, and " */
    /*                                      << approachResult.update_method << " update method was used."); */
    /* } */
    /* else */
    /* { */
    /* RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution"); */
    /* } */

    // Visualize and execute trajectory
    /* bool success = visualize_and_execute_trajectory_(solution, robot_start_config, aff.axis, aff.location); */
    /* bool success = visualize_and_execute_trajectory_(solution, Eigen::VectorXd::Zero(6), aff.axis, aff.location); */
    /* return visualize_and_execute_trajectory_(solution, robot_start_config, aff.axis, aff.location); */
}

std::vector<Eigen::VectorXd> CcAffordancePlannerRos::run_cc_affordance_planner_affordance_motion(
    const cc_affordance_planner::PlannerConfig &plannerConfig, affordance_util::ScrewInfo &aff,
    const Eigen::VectorXd &sec_goal, const size_t &gripper_control_par, const std::string &vir_screw_order,
    Eigen::VectorXd robot_start_config)
{
    // If tag frame is specified then, we lookup affordance location from tag
    /* if (!aff.location_frame.empty()) */
    /* { */

    /*     RCLCPP_INFO_STREAM(node_logger_, "Ready to read affordance location from apriltag? y or Y for yes."); */
    /*     std::string conf; */
    /*     std::cin >> conf; */
    /*     if (conf != "y" && conf != "Y") */
    /*     { */
    /*         throw std::runtime_error("You indicated you are not ready to read affordance location"); */
    /*     } */
    /*     const Eigen::Isometry3d aff_htm = affordance_util_ros::get_htm(ref_frame_, aff.location_frame, *tf_buffer_);
     */
    /*     /1* aff.location = aff_htm.translation(); *1/ */
    /* } */
    const Eigen::Matrix<double, 6, 1> aff_screw = affordance_util::get_screw(aff); // compute affordance screw

    // Get joint states at the start configuration of the affordance
    if (robot_start_config.size() == 0) // Non-zero if testing or planning without the joint_states topic
    {
        robot_start_config = get_aff_start_joint_states_();
    }

    // Compose cc model and affordance goal
    Eigen::MatrixXd cc_slist =
        affordance_util::compose_cc_model_slist(robot_slist_, robot_start_config, M_, aff_screw, vir_screw_order);

    // Run the planner
    cc_affordance_planner::PlannerResult plannerResult =
        cc_affordance_planner::generate_joint_trajectory(plannerConfig, cc_slist, sec_goal, gripper_control_par);

    // Print planner result
    std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
    if (plannerResult.success)
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with "
                                             << plannerResult.traj_full_or_partial << " solution, planning took "
                                             << plannerResult.planning_time.count() << " microseconds, and "
                                             << plannerResult.update_method << " update method was used.");
        bool success = visualize_and_execute_trajectory_(solution, robot_start_config, aff.axis, aff.location);
        return plannerResult.joint_traj;
    }
    else
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
    }

    // Visualize and execute trajectory
    /* return visualize_and_execute_trajectory_(solution, robot_start_config, aff.axis, aff.location); */
}

bool CcAffordancePlannerRos::run_cc_affordance_planner(const cc_affordance_planner::PlannerConfig &plannerConfig,
                                                       affordance_util::ScrewInfo &aff, const Eigen::VectorXd &sec_goal,
                                                       const size_t &gripper_control_par,
                                                       const std::string &vir_screw_order,
                                                       Eigen::VectorXd robot_start_config)
{
    // If tag frame is specified then, we lookup affordance location from tag
    if (!aff.location_frame.empty())
    {

        RCLCPP_INFO_STREAM(node_logger_, "Ready to read affordance location from apriltag? y or Y for yes.");
        std::string conf;
        std::cin >> conf;
        if (conf != "y" && conf != "Y")
        {
            throw std::runtime_error("You indicated you are not ready to read affordance location");
        }
        const Eigen::Isometry3d aff_htm = affordance_util_ros::get_htm(ref_frame_, aff.location_frame, *tf_buffer_);
        aff.location = aff_htm.translation();
    }
    const Eigen::Matrix<double, 6, 1> aff_screw = affordance_util::get_screw(aff); // compute affordance screw

    // Get joint states at the start configuration of the affordance
    if (robot_start_config.size() == 0) // Non-zero if testing or planning without the joint_states topic
    {
        robot_start_config = get_aff_start_joint_states_();
    }

    // Compose cc model and affordance goal
    Eigen::MatrixXd cc_slist =
        affordance_util::compose_cc_model_slist(robot_slist_, robot_start_config, M_, aff_screw, vir_screw_order);

    // Run the planner
    cc_affordance_planner::PlannerResult plannerResult =
        cc_affordance_planner::generate_joint_trajectory(plannerConfig, cc_slist, sec_goal, gripper_control_par);

    // Print planner result
    std::vector<Eigen::VectorXd> solution = plannerResult.joint_traj;
    if (plannerResult.success)
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner succeeded with "
                                             << plannerResult.traj_full_or_partial << " solution, planning took "
                                             << plannerResult.planning_time.count() << " microseconds, and "
                                             << plannerResult.update_method << " update method was used.");
    }
    else
    {
        RCLCPP_INFO_STREAM(node_logger_, "Planner did not find a solution");
    }

    // Visualize and execute trajectory
    return visualize_and_execute_trajectory_(solution, robot_start_config, aff.axis, aff.location);
}

// Returns full path to the yaml file containing cc affordance robot description
std::string CcAffordancePlannerRos::get_cc_affordance_robot_description_(const std::string &robot_name)
{
    const std::string package_name = "cca_" + robot_name;            // get package name from the parameter server
    const std::string rel_dir = "/config/";                          // relative directory where yaml file is located
    const std::string filename = package_name + "_description.yaml"; // yaml file name
    return affordance_util_ros::get_filepath_inside_pkg(package_name, rel_dir, filename);
}

// Callback function for the joint_states subscriber
void CcAffordancePlannerRos::joint_states_cb_(const JointState::SharedPtr msg)
{
    joint_states_ = affordance_util_ros::get_ordered_joint_states(
        msg,
        joint_names_); // Takes care of filtering and ordering the joint_states
}

// Function to read robot joint states at the start of the affordance
Eigen::VectorXd CcAffordancePlannerRos::get_aff_start_joint_states_()
{
    // Set Eigen::VectorXd size
    joint_states_.positions.conservativeResize(joint_names_.size());

    // Capture initial configuration joint states
    RCLCPP_INFO_STREAM(node_logger_,
                       "Put the robot in the start configuration for affordance execution. Done? y for yes.");
    std::string capture_joint_states_conf;
    std::cin >> capture_joint_states_conf;

    if (capture_joint_states_conf != "y" && capture_joint_states_conf != "Y")
    {
        throw std::runtime_error("You indicated you are not ready to capture joint states");
    }

    return joint_states_.positions;
}

// Function to visualize and execute planned trajectory
bool CcAffordancePlannerRos::visualize_and_execute_trajectory_(const std::vector<Eigen::VectorXd> &trajectory,
                                                               const Eigen::VectorXd &robot_start_config,
                                                               const Eigen::VectorXd &w_aff,
                                                               const Eigen::VectorXd &q_aff)
{

    // Visualize trajectory in RVIZ
    // Convert the solution trajectory to ROS message type
    /* const double traj_time_step = 0.8; */
    const double traj_time_step = 0.4;
    const control_msgs::action::FollowJointTrajectory_Goal goal =
        affordance_util_ros::follow_joint_trajectory_msg_builder(
            trajectory, robot_start_config, joint_names_,
            traj_time_step); // this function takes care of extracting the right
                             // number of joint_states although solution
                             // contains qs data too

    auto end_point = goal.trajectory.points[goal.trajectory.points.size() - 1].positions;

    // Convert std::vector<double> to Eigen::VectorXd using Eigen::Map
    Eigen::VectorXd end_point_vec(end_point.size());
    std::copy(end_point.begin(), end_point.end(), end_point_vec.data());
    const Eigen::Matrix4d computed_grasp_pose = affordance_util::FKinSpace(M_, robot_slist_, end_point_vec);
    std::cout << "Here is the computed grasp pose:\n" << computed_grasp_pose << std::endl;
    // Fill out service request
    auto plan_and_viz_serv_req = std::make_shared<MoveItPlanAndViz::Request>();
    plan_and_viz_serv_req->joint_traj = goal.trajectory;
    plan_and_viz_serv_req->aff_screw_axis = {w_aff[0], w_aff[1], w_aff[2]};
    plan_and_viz_serv_req->aff_location = {q_aff[0], q_aff[1], q_aff[2]};
    plan_and_viz_serv_req->ref_frame = ref_frame_;
    plan_and_viz_serv_req->tool_frame = tool_frame_;
    plan_and_viz_serv_req->planning_group = planning_group_;
    plan_and_viz_serv_req->robot_description = robot_description_parameter_;
    plan_and_viz_serv_req->rviz_fixed_frame = rviz_fixed_frame_;

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
        RCLCPP_INFO_STREAM(node_logger_, "Ready to execute the trajectory? y to confirm");
        std::string execution_conf;
        std::cin >> execution_conf;

        if (execution_conf != "y" && execution_conf != "Y")
        {
            RCLCPP_INFO_STREAM(node_logger_, "Trajectory execution was canceled");
            return false;
        }

        // Send the goal to follow_joint_trajectory action server for execution
        if (!this->traj_execution_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_logger_, " %s action server not available after waiting",
                         traj_execution_as_name_.c_str());
            return false;
        }

        RCLCPP_INFO(node_logger_, "Sending goal to %s action server", traj_execution_as_name_.c_str());

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_goal_response_callback_, this, std::placeholders::_1);
        /* send_goal_options.feedback_callback =
         * std::bind(&CcAffordancePlannerRos::traj_execution_feedback_callback_,
         */
        /*                                                 this, std::placeholders::_1, std::placeholders::_2); */
        send_goal_options.result_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_result_callback_, this, std::placeholders::_1);

        this->traj_execution_client_->async_send_goal(goal, send_goal_options);
        return true;
    }
    else
    {
        RCLCPP_ERROR(node_logger_, "%s service call failed", plan_and_viz_ss_name_.c_str());
        // Execute trajectory on the real robot
        RCLCPP_INFO_STREAM(node_logger_, "Ready to execute the trajectory? y to confirm");
        std::string execution_conf;
        std::cin >> execution_conf;

        if (execution_conf != "y" && execution_conf != "Y")
        {
            RCLCPP_INFO_STREAM(node_logger_, "Trajectory execution was canceled");
            return false;
        }

        // Send the goal to follow_joint_trajectory action server for execution
        if (!this->traj_execution_client_->wait_for_action_server())
        {
            RCLCPP_ERROR(node_logger_, " %s action server not available after waiting",
                         traj_execution_as_name_.c_str());
            return false;
        }

        RCLCPP_INFO(node_logger_, "Sending goal to %s action server", traj_execution_as_name_.c_str());

        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_goal_response_callback_, this, std::placeholders::_1);
        /* send_goal_options.feedback_callback =
         * std::bind(&CcAffordancePlannerRos::traj_execution_feedback_callback_,
         */
        /*                                                 this, std::placeholders::_1, std::placeholders::_2); */
        send_goal_options.result_callback =
            std::bind(&CcAffordancePlannerRos::traj_execution_result_callback_, this, std::placeholders::_1);

        this->traj_execution_client_->async_send_goal(goal, send_goal_options);
        return false;
    }
}

// Callback to process traj_execution_as feedback
/* void traj_execution_feedback_callback_(GoalHandleFollowJointTrajectory::SharedPtr, */
/*                                        const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) */
/* { */
/*     // Empty for now */
/* } */

// Callback to process traj_execution_as result
void CcAffordancePlannerRos::traj_execution_result_callback_(
    const GoalHandleFollowJointTrajectory::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_logger_, "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_logger_, "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(node_logger_, "Unknown result code");
        return;
    }
    RCLCPP_INFO(node_logger_, "%s action server call concluded", traj_execution_as_name_.c_str());
    /* rclcpp::shutdown(); */
}

// Callback to process traj_exection_as goal response
void CcAffordancePlannerRos::traj_execution_goal_response_callback_(
    const GoalHandleFollowJointTrajectory::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_ERROR(node_logger_, "Goal was rejected by %s action server", traj_execution_as_name_.c_str());
    }
    else
    {
        RCLCPP_INFO(node_logger_, "Goal accepted by %s action server, waiting for result",
                    traj_execution_as_name_.c_str());
    }
}
