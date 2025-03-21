#include "cca_interactive_goals/panel.hpp"

namespace cca_interactive_goals
{

CcaInteractiveGoals::CcaInteractiveGoals(QWidget *parent)
    : rviz_common::Panel(parent), interactive_marker_manager::InteractiveMarkerManager("cca_interactive_goals")
{
    QTabWidget *tabWidget = new QTabWidget(this);

    // CCA_IG Tab
    QWidget *ccaIgTabWidget = new QWidget();
    QVBoxLayout *mainLayout = new QVBoxLayout;

    // Mode selection dropdown
    QHBoxLayout *mode_layout = new QHBoxLayout;
    QLabel *mode_label = new QLabel("Select Mode:");
    mode_combo_box_ = new QComboBox;
    mode_combo_box_->addItems({"Affordance Planning", "In-Place End Effector Orientation Control"});
    mode_layout->addWidget(mode_label);
    mode_layout->addWidget(mode_combo_box_);
    mainLayout->addLayout(mode_layout);

    // Dynamic content layout
    QVBoxLayout *dynamicContentLayout = new QVBoxLayout;
    dynamicContentLayout->setSpacing(10); // Set spacing between dynamic boxes

    // Motion type dropdown
    QHBoxLayout *motion_type_layout = new QHBoxLayout;
    motion_type_label_ = new QLabel("Motion Type:");
    motion_type_combo_box_ = new QComboBox;
    motion_type_combo_box_->addItems({"", "Translation", "Rotation", "Screw Motion"});
    motion_type_layout->addWidget(motion_type_label_);
    motion_type_layout->addWidget(motion_type_combo_box_);
    dynamicContentLayout->addLayout(motion_type_layout);

    // Axis Selection Dropdown
    QHBoxLayout *axis_layout = new QHBoxLayout;
    axis_label_ = new QLabel("Axis:");
    axis_combo_box_ = new QComboBox;
    axis_combo_box_->addItems({"", "Interactive Axis", "x", "y", "z", "-x", "-y", "-z"});
    axis_layout->addWidget(axis_label_);
    axis_layout->addWidget(axis_combo_box_);
    dynamicContentLayout->addLayout(axis_layout);

    // Pitch Selection Dropdown
    QHBoxLayout *pitch_layout = new QHBoxLayout;
    pitch_label_ = new QLabel("Pitch(meters/radian):");
    pitch_combo_box_ = new QComboBox;
    pitch_combo_box_->addItems({"", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5"});
    pitch_layout->addWidget(pitch_label_);
    pitch_layout->addWidget(pitch_combo_box_);
    dynamicContentLayout->addLayout(pitch_layout);

    // Pitch input
    QHBoxLayout *pitch_value_layout = new QHBoxLayout;
    pitch_value_label_ = new QLabel("Pitch Value(meters/radian):");
    pitch_value_label_->setObjectName("Pitch Value Label");
    pitch_value_input_ = new QLineEdit;
    pitch_value_layout->addWidget(pitch_value_label_);
    pitch_value_layout->addWidget(pitch_value_input_);
    dynamicContentLayout->addLayout(pitch_value_layout);

    // Goal Selection Dropdown
    QHBoxLayout *goal_layout = new QHBoxLayout;
    goal_label_ = new QLabel("Goal:");
    goal_combo_box_ = new QComboBox;
    goal_combo_box_->addItems(
        {"", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6", "0.7", "0.8", "0.9", "1.0"});
    goal_layout->addWidget(goal_label_);
    goal_layout->addWidget(goal_combo_box_);
    dynamicContentLayout->addLayout(goal_layout);

    // Goal input
    QHBoxLayout *value_layout = new QHBoxLayout;
    value_label_ = new QLabel("Value:");
    value_label_->setObjectName("Value Label");
    value_input_ = new QLineEdit;
    value_layout->addWidget(value_label_);
    value_layout->addWidget(value_input_);
    dynamicContentLayout->addLayout(value_layout);

    mainLayout->addLayout(dynamicContentLayout);

    // Spacer to push buttons to the bottom
    mainLayout->addStretch();

    // Plan, Viz, Execute Buttons
    QVBoxLayout *buttonLayout = new QVBoxLayout;
    plan_viz_button_ = new QPushButton("Plan");
    buttonLayout->addWidget(plan_viz_button_);
    plan_viz_button_->setEnabled(false);
    plan_viz_exe_button_ = new QPushButton("Plan and Execute");
    buttonLayout->addWidget(plan_viz_exe_button_);
    plan_viz_exe_button_->setEnabled(false);
    plan_exe_button_ = new QPushButton("Execute");
    buttonLayout->addWidget(plan_exe_button_);
    plan_exe_button_->setEnabled(false);
    stop_button_ = new QPushButton("Cancel Execution");
    buttonLayout->addWidget(stop_button_);
    stop_button_->setEnabled(false);

    mainLayout->addLayout(buttonLayout);

    ccaIgTabWidget->setLayout(mainLayout);

    // Advanced Settings Tab
    QWidget *advancedSettingsTabWidget = new QWidget();
    QVBoxLayout *advancedLayout = new QVBoxLayout;
    QFormLayout *form_layout = new QFormLayout;

    // Text input fields
    accuracy_ = new QLineEdit();
    closure_angle_ = new QLineEdit();
    closure_linear_ = new QLineEdit();
    ik_iterations_ = new QLineEdit();
    trajectory_density_ = new QLineEdit();

    // Dropdown menus
    screw_order_combo_ = new QComboBox();

    // Create a QStringList from virtual_screw_order_map_
    QStringList screw_options;
    for (const auto &pair : virtual_screw_order_map_)
    {
        screw_options.append(pair.first);
    }

    // Add the screw options to the QComboBox
    screw_order_combo_->addItem(QString(""));
    screw_order_combo_->addItems(screw_options);

    // Set XYZ as default option for screw order
    screw_order_combo_->setCurrentText(QString(""));

    cca_type_combo_ = new QComboBox();
    cca_type_combo_->addItems({"Affordance Control", "Affordance and EE Control"});

    form_layout->addRow("Accuracy:", accuracy_);
    form_layout->addRow("Closure Error Threshold Angle:", closure_angle_);
    form_layout->addRow("Closure Error Threshold Linear:", closure_linear_);
    form_layout->addRow("IK Max Iterations:", ik_iterations_);
    form_layout->addRow("Trajectory Density:", trajectory_density_);
    form_layout->addRow("Virtual Screw Order:", screw_order_combo_);
    form_layout->addRow("CCA Type:", cca_type_combo_);

    // Add form layout to advanced tab
    advancedLayout->addLayout(form_layout);
    advancedLayout->addStretch();

    apply_button_ = new QPushButton("Apply Settings");

    QVBoxLayout *button_layout = new QVBoxLayout();
    button_layout->addWidget(apply_button_);

    advancedLayout->addLayout(button_layout); //

    advancedSettingsTabWidget->setLayout(advancedLayout);

    tabWidget->addTab(ccaIgTabWidget, "CCA_IG");
    tabWidget->addTab(advancedSettingsTabWidget, "Advanced Settings");

    QVBoxLayout *panelLayout = new QVBoxLayout;
    panelLayout->addWidget(tabWidget);
    setLayout(panelLayout);

    setFixedSize(400, 475); // Set fixed size for the panel

    // Linking of the callback functions

    connect(mode_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(modeSelected(int)));
    connect(motion_type_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(motionTypeSelected(int)));
    connect(goal_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(goalSelected(int)));
    connect(axis_combo_box_, SIGNAL(currentTextChanged(QString)), this, SLOT(axisOptionSelected(QString)));
    connect(pitch_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(pitchSelected(int)));
    connect(plan_viz_button_, SIGNAL(clicked()), this, SLOT(planVizClicked()));
    connect(plan_viz_exe_button_, SIGNAL(clicked()), this, SLOT(planVizExeClicked()));
    connect(plan_exe_button_, SIGNAL(clicked()), this, SLOT(planExeClicked()));
    connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopClicked()));

    connect(apply_button_, SIGNAL(clicked()), this, SLOT(applySettingsClicked()));

    updateUIState();

    // Interactive marker initialization
}

void CcaInteractiveGoals::onInitialize()
{
    // Do ROS client initializations here
    ccaRosActionClient = std::make_shared<cca_ros_action::CcaRosActionClient>();

    // Hide the markers to start
    this->hide_im("arrow_marker");

    // Set up timer for spinning the node
    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, &CcaInteractiveGoals::spin);
    spin_timer_->start(10); // Spin every 10ms
}

void CcaInteractiveGoals::load(const rviz_common::Config &config) { rviz_common::Panel::load(config); }

void CcaInteractiveGoals::save(rviz_common::Config config) const { rviz_common::Panel::save(config); }

void CcaInteractiveGoals::planVizClicked()
{
    // Handle plan viz button click
    auto req = buildPlanningRequest();
    stop_button_->setEnabled(true);

    RCLCPP_INFO(this->get_logger(), "planViz button pressed");
    req.visualize_trajectory = true;
    req.execute_trajectory = false;
    ccaRosActionClient->send_goal(req);
}

void CcaInteractiveGoals::planVizExeClicked()
{
    // Handle plan viz execute button click
    auto req = buildPlanningRequest();
    stop_button_->setEnabled(true);

    RCLCPP_INFO(this->get_logger(), "planVizExe button pressed");
    req.visualize_trajectory = true;
    req.execute_trajectory = true;
    ccaRosActionClient->send_goal(req);
}

void CcaInteractiveGoals::planExeClicked()
{
    // Handle plan execute button click
    auto req = buildPlanningRequest();
    stop_button_->setEnabled(true);

    RCLCPP_INFO(this->get_logger(), "planExe button pressed");
    req.visualize_trajectory = false;
    req.execute_trajectory = true;
    ccaRosActionClient->send_goal(req);
}

void CcaInteractiveGoals::stopClicked() { ccaRosActionClient->cancel_goal(); }

cca_ros::PlanningRequest CcaInteractiveGoals::buildPlanningRequest()
{

    cca_ros::PlanningRequest req;

    // Set planning request start state for testing purposes
    const Eigen::VectorXd READY_CONFIG =
        (Eigen::VectorXd(6) << -0.00015592575073242188, -0.8980185389518738, 1.8094338178634644, 0.000377655029296875,
         -0.8991076946258545, 0.0015475749969482422)
            .finished();
    req.start_state.robot = READY_CONFIG;
    if (mode_combo_box_->currentText() == "Affordance Planning") // Affordance Planning
    {
        req.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

        // Determine motion type for affordance planning
        if (motion_type_combo_box_->currentText() == "Translation")
        {
            req.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;
        }
        else if (motion_type_combo_box_->currentText() == "Rotation")
        {
            req.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
        }
        else if (motion_type_combo_box_->currentText() == "Screw Motion")
        {
            req.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;

            // Determine pitch value
            if (pitch_combo_box_->currentText() != "Manual Input")
            {
                req.task_description.affordance_info.pitch = std::stof(pitch_combo_box_->currentText().toStdString());
            }
            else // Manual input
            {
                try
                {
                    req.task_description.affordance_info.pitch = std::stof(pitch_value_input_->text().toStdString());
                }
                catch (int)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Screw Pitch could not be converted to type float. Please check entry and try again");
                }
            }
        }
    }
    else if (mode_combo_box_->currentText() == "In-Place End Effector Orientation Control")
    {
        req.task_description =
            cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);
    }

    // Get affordance goal
    req.task_description.goal.affordance = getAffordanceGoal_();

    // Get affordance pose
    auto screw_info = this->get_arrow_pose(mode_combo_box_->currentText().toStdString(),
                                           axis_combo_box_->currentText().toStdString());
    req.task_description.affordance_info.axis = screw_info.axis;
    req.task_description.affordance_info.location = screw_info.location;

    // Extract task-specific settingsa from advanced settings
    if (new_settings_applied_)
    {
        req.planner_config = advanced_settings_.planner_config;
        req.task_description.trajectory_density = advanced_settings_.task_description.trajectory_density;
        if (advanced_settings_.task_description.vir_screw_order
                .has_value()) // Since the default for this is different for different planning types, we
                              // check if the user has specified a value before applying it. For all other settings, the
                              // default is the same across all planning types
        {
            req.task_description.vir_screw_order = advanced_settings_.task_description.vir_screw_order.value();
        }
    }
    return req;
}

double CcaInteractiveGoals::getAffordanceGoal_()
{
    RCLCPP_INFO(this->get_logger(), "Getting affordance goal");
    double goal;
    if ((motion_type_combo_box_->currentText() == "Rotation" ||
         motion_type_combo_box_->currentText() == "Screw Motion" ||
         mode_combo_box_->currentText() == "In-Place End Effector Orientation Control") &&
        (goal_combo_box_->currentText() != "Manual Input"))
    {
        goal = M_PI * (goal_combo_box_->currentIndex() - 1) / 4.0; // multiple of pi/4
    }
    else if (motion_type_combo_box_->currentText() == "Translation" && goal_combo_box_->currentText() != "Manual Input")
    {
        goal = std::stof(goal_combo_box_->currentText().toStdString());
    }
    else
    { // Manual Input
        try
        {
            goal = std::stof(value_input_->text().toStdString());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Unexpected error occurred during conversion: %s", e.what());
            goal = 0.0;
        }
        catch (...)
        {
            RCLCPP_ERROR(this->get_logger(), "An unknown error occurred during conversion.");
            goal = 0.0;
        }
    }
    return goal;
}

void CcaInteractiveGoals::modeSelected(int index)
{
    // Grey out execute buttons
    plan_viz_button_->setEnabled(false);
    plan_viz_exe_button_->setEnabled(false);
    plan_exe_button_->setEnabled(false);
    stop_button_->setEnabled(false);

    // Setup other widgets based on selection

    bool mode_selected = mode_combo_box_->currentIndex() != -1;

    this->hide_im("arrow_marker");

    if (mode_selected)
    {
        if (mode_combo_box_->currentText() == "Affordance Planning")
        {

            axis_combo_box_->setEnabled(false);
            axis_combo_box_->setVisible(false);
            axis_label_->setVisible(false);
            goal_label_->setVisible(false);
            goal_combo_box_->setEnabled(false);
            goal_combo_box_->setVisible(false);
            value_input_->setVisible(false);
            value_input_->setEnabled(false);
            value_label_->setVisible(false);
            pitch_label_->setVisible(false);
            pitch_combo_box_->setVisible(false);
            pitch_value_input_->setVisible(false);
            pitch_value_label_->setVisible(false);

            // Set necessary widgets to visible
            motion_type_label_->setVisible(true);
            motion_type_combo_box_->setEnabled(true);
            motion_type_combo_box_->setVisible(true);
            motion_type_combo_box_->setCurrentText("");
        }
        else if (mode_combo_box_->currentText() == "In-Place End Effector Orientation Control")
        {
            motion_type_label_->setVisible(false);
            motion_type_combo_box_->setEnabled(false);
            motion_type_combo_box_->setVisible(false);
            goal_label_->setVisible(false);
            goal_combo_box_->setEnabled(false);
            goal_combo_box_->setVisible(false);
            value_input_->setVisible(false);
            value_input_->setEnabled(false);
            value_label_->setVisible(false);
            pitch_label_->setVisible(false);
            pitch_combo_box_->setVisible(false);
            pitch_value_input_->setVisible(false);
            pitch_value_label_->setVisible(false);

            // Set necessary widgets to visible

            axis_combo_box_->setEnabled(true);
            axis_combo_box_->setVisible(true);
            axis_label_->setVisible(true);
            axis_combo_box_->setCurrentText("");
        }
    }
}

void CcaInteractiveGoals::motionTypeSelected(int index)
{
    plan_exe_button_->setEnabled(false);
    plan_viz_button_->setEnabled(false);
    plan_viz_exe_button_->setEnabled(false);
    value_input_->setVisible(false);
    value_input_->setEnabled(false);
    value_label_->setVisible(false);
    pitch_label_->setVisible(false);
    pitch_combo_box_->setVisible(false);
    pitch_value_input_->setVisible(false);
    pitch_value_label_->setVisible(false);
    pitch_combo_box_->setCurrentIndex(0);
    if (motion_type_combo_box_->currentText() == "Translation" || motion_type_combo_box_->currentText() == "Rotation" ||
        motion_type_combo_box_->currentText() == "Screw Motion")
    {
        this->enable_im_controls("arrow_marker", interactive_marker_manager::ImControlEnable::ALL);
    }
    else
    {
        this->hide_im("arrow_marker");
    }
    if (motion_type_combo_box_->currentText() == "Screw Motion")
    {
        pitch_label_->setVisible(true);
        pitch_combo_box_->setVisible(true);
        pitch_value_input_->setVisible(false);
        pitch_value_label_->setVisible(false);
    }
    // Enable goal boxes
    goal_label_->setVisible(true);
    goal_combo_box_->setEnabled(true);
    goal_combo_box_->setVisible(true);
    goal_combo_box_->setCurrentIndex(0);
    if (motion_type_combo_box_->currentText() == "Translation")
    {
        goal_label_->setText("Goal Distance(meters)");
        for (int i = 2; i <= 11; i++)
        {
            double value = (i - 1) * 0.1;
            QString text = QString::number(value, 'f', 1);
            goal_combo_box_->setItemText(i, text);
        }
    }
    else if (motion_type_combo_box_->currentText() == "Rotation" ||
             motion_type_combo_box_->currentText() == "Screw Motion")
    {
        goal_label_->setText("Goal Angle(radians)");
        std::vector<std::string> pi_fractions = {"π/4",  "π/2",  "3π/4", "π",    "5π/4",
                                                 "3π/2", "7π/4", "2π",   "9π/4", "5π/2"};
        for (int i = 2; i <= 11; i++)
        {
            goal_combo_box_->setItemText(i, pi_fractions[i - 2].c_str());
        }
    }
}

void CcaInteractiveGoals::goalSelected(int index)
{
    if (index == 1)
    {
        value_label_->setVisible(true);
        value_input_->setEnabled(true);
        value_input_->setVisible(true);
        if (motion_type_combo_box_->currentText() == "Translation" &&
            mode_combo_box_->currentText() != "In-Place End Effector Orientation Control")
        {
            value_label_->setText("Meters");
        }
        else if (motion_type_combo_box_->currentText() == "Rotation" ||
                 motion_type_combo_box_->currentText() == "Screw Motion" ||
                 mode_combo_box_->currentText() == "In-Place End Effector Orientation Control")
        {
            value_label_->setText("Radians");
        }
    }
    else
    {
        value_label_->setVisible(false);
        value_input_->setEnabled(false);
        value_input_->setVisible(false);
    }
    if (index != 0 &&
        (motion_type_combo_box_->currentText() != "Screw Motion" || pitch_combo_box_->currentIndex() != 0))
    {
        plan_exe_button_->setEnabled(true);
        plan_viz_button_->setEnabled(true);
        plan_viz_exe_button_->setEnabled(true);
    }
    else
    {
        plan_exe_button_->setEnabled(false);
        plan_viz_button_->setEnabled(false);
        plan_viz_exe_button_->setEnabled(false);
    }
}

void CcaInteractiveGoals::pitchSelected(int index)
{
    if (index == 1)
    {
        pitch_value_input_->setVisible(true);
        pitch_value_label_->setVisible(true);
    }
    else
    {
        pitch_value_input_->setVisible(false);
        pitch_value_label_->setVisible(false);
    }
    if (index != 0 && goal_combo_box_->currentIndex() != 0)
    {
        plan_exe_button_->setEnabled(true);
        plan_viz_button_->setEnabled(true);
        plan_viz_exe_button_->setEnabled(true);
    }
    else
    {
        plan_exe_button_->setEnabled(false);
        plan_viz_button_->setEnabled(false);
        plan_viz_exe_button_->setEnabled(false);
    }
}

void CcaInteractiveGoals::axisOptionSelected(QString axis)
{
    if (axis != "")
    {
        goal_label_->setVisible(true);
        goal_combo_box_->setEnabled(true);
        goal_combo_box_->setVisible(true);
        goal_combo_box_->setCurrentIndex(0);
        goal_label_->setText("Goal Angle(radians)");
        std::vector<std::string> pi_fractions = {"π/4",  "π/2",  "3π/4", "π",    "5π/4",
                                                 "3π/2", "7π/4", "2π",   "9π/4", "5π/2"};
        for (int i = 2; i <= 11; i++)
        {
            goal_combo_box_->setItemText(i, pi_fractions[i - 2].c_str());
        }
    }

    this->draw_ee_or_control_im(axis.toStdString());
}

void CcaInteractiveGoals::applySettingsClicked()
{
    AdvancedSettings advanced_settings;

    if (accuracy_->text().toFloat() != 0)
    {
        advanced_settings.planner_config.accuracy = accuracy_->text().toFloat();
    }

    if (closure_angle_->text().toFloat() != 0)
    {
        advanced_settings.planner_config.closure_err_threshold_ang = closure_angle_->text().toFloat();
    }

    if (closure_linear_->text().toFloat() != 0)
    {
        advanced_settings.planner_config.closure_err_threshold_lin = closure_linear_->text().toFloat();
    }

    if (ik_iterations_->text().toInt() != 0)
    {
        advanced_settings.planner_config.ik_max_itr = ik_iterations_->text().toInt();
    }
    if (trajectory_density_->text().toInt() != 0)
    {
        advanced_settings.task_description.trajectory_density = trajectory_density_->text().toInt();
    }

    if (screw_order_combo_->currentIndex() != 0)
    {
        advanced_settings.task_description.vir_screw_order =
            virtual_screw_order_map_.at(screw_order_combo_->currentText());
    }

    advanced_settings_ = advanced_settings;
    new_settings_applied_ = true;
    RCLCPP_INFO(this->get_logger(), "New Advanced Settings Applied");
}

void CcaInteractiveGoals::updateUIState()
{
    plan_viz_button_->setEnabled(false);
    plan_viz_exe_button_->setEnabled(false);
    plan_exe_button_->setEnabled(false);
    stop_button_->setEnabled(false);
    axis_combo_box_->setEnabled(false);
    axis_combo_box_->setVisible(false);
    axis_label_->setVisible(false);
    goal_label_->setVisible(false);
    goal_combo_box_->setEnabled(false);
    goal_combo_box_->setVisible(false);
    value_input_->setVisible(false);
    value_input_->setEnabled(false);
    value_label_->setVisible(false);
    pitch_label_->setVisible(false);
    pitch_combo_box_->setVisible(false);
    pitch_value_input_->setVisible(false);
    pitch_value_label_->setVisible(false);

    // Set necessary widgets to visible
    motion_type_label_->setVisible(true);
    motion_type_combo_box_->setEnabled(true);
    motion_type_combo_box_->setVisible(true);
    motion_type_combo_box_->setCurrentText("");
}

void CcaInteractiveGoals::spin() { rclcpp::spin_some(this->get_node_base_interface()); }

} // namespace cca_interactive_goals
// TODO: get marker frame id from outside to make this robot-agnostic
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cca_interactive_goals::CcaInteractiveGoals, rviz_common::Panel)
