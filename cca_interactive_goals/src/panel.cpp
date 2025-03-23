#include "cca_interactive_goals/panel.hpp"

namespace cca_interactive_goals
{
CcaInteractiveGoals::CcaInteractiveGoals(QWidget *parent)
    : rviz_common::Panel(parent), interactive_marker_manager::InteractiveMarkerManager("cca_interactive_goals")
{
    auto *tab_widget = new QTabWidget(this);

    tab_widget->addTab(this->create_cca_ig_tab_(), "CCA_IG");
    tab_widget->addTab(this->create_advanced_settings_tab_(), "Advanced Settings");

    auto *panel_layout = new QVBoxLayout;
    panel_layout->addWidget(tab_widget);
    setLayout(panel_layout);

    setFixedSize(PANEL_WIDTH_, PANEL_HEIGHT_);

    this->connect_signals_();
    this->update_ui_state_();
}

QWidget *CcaInteractiveGoals::create_cca_ig_tab_()
{
    auto *cca_ig_tab_widget = new QWidget();
    auto *main_layout = new QVBoxLayout(cca_ig_tab_widget);

    main_layout->addLayout(
        this->create_combo_box_layout_("Planning Type:", mode_bl_, this->get_map_keys_(planning_type_map_)));
    auto *dynamic_content_layout = new QVBoxLayout;
    dynamic_content_layout->setSpacing(LAYOUT_SPACING_);

    dynamic_content_layout->addLayout(
        this->create_combo_box_layout_("Motion Type:", motion_type_bl_, this->get_map_keys_(motion_type_map_, true)));
    dynamic_content_layout->addLayout(this->create_combo_box_layout_("Axis:", axis_bl_, AXES_));
    dynamic_content_layout->addLayout(this->create_combo_box_layout_("Pitch(meters/radian):", pitch_bl_, PITCHES_));
    dynamic_content_layout->addLayout(this->create_line_edit_layout_("Pitch Value(meters/radian):", pitch_value_ll_));
    dynamic_content_layout->addLayout(this->create_combo_box_layout_("Goal:", goal_bl_, TRANSLATION_GOALS_));
    dynamic_content_layout->addLayout(this->create_line_edit_layout_("Value:", value_ll_));

    main_layout->addLayout(dynamic_content_layout);
    main_layout->addStretch();

    auto *button_layout = new QVBoxLayout;
    this->create_button_(plan_viz_button_, "Plan", button_layout);
    this->create_button_(plan_viz_exe_button_, "Plan and Execute", button_layout);
    this->create_button_(plan_exe_button_, "Execute", button_layout);
    this->create_button_(stop_button_, "Cancel Execution", button_layout);

    main_layout->addLayout(button_layout);
    return cca_ig_tab_widget;
}

QWidget *CcaInteractiveGoals::create_advanced_settings_tab_()
{
    auto *advanced_settings_tab_widget = new QWidget();
    auto *advanced_layout = new QVBoxLayout(advanced_settings_tab_widget);

    auto *form_layout = new QFormLayout;

    // Here we connect the QLineEdits to the widgets struct
    advanced_settings_widgets_.accuracy = new QLineEdit();
    advanced_settings_widgets_.closure_angle = new QLineEdit();
    advanced_settings_widgets_.closure_linear = new QLineEdit();
    advanced_settings_widgets_.ik_iterations = new QLineEdit();
    advanced_settings_widgets_.trajectory_density = new QLineEdit();

    form_layout->addRow("Accuracy:", advanced_settings_widgets_.accuracy);
    form_layout->addRow("Closure Error Threshold Angle:", advanced_settings_widgets_.closure_angle);
    form_layout->addRow("Closure Error Threshold Linear:", advanced_settings_widgets_.closure_linear);
    form_layout->addRow("IK Max Iterations:", advanced_settings_widgets_.ik_iterations);
    form_layout->addRow("Trajectory Density:", advanced_settings_widgets_.trajectory_density);

    form_layout->addRow(create_combo_box_layout_("Virtual Screw Order:", advanced_settings_widgets_.vir_screw_order,
                                                 this->get_map_keys_(vir_screw_order_map_, true)));
    form_layout->addRow(
        create_combo_box_layout_("CCA Type:", advanced_settings_widgets_.cca_type, this->get_map_keys_(cca_type_map_)));

    advanced_layout->addLayout(form_layout);
    advanced_layout->addStretch();

    apply_button_ = new QPushButton("Apply Settings");
    advanced_layout->addWidget(apply_button_);

    return advanced_settings_tab_widget;
}

void CcaInteractiveGoals::update_ui_state_()
{
    // Disable all execution buttons
    set_execute_buttons_enabled_(false);

    // Hide all controls first
    hide_all_controls_();

    // Set only motion_type controls to visible
    set_combo_box_controls_(motion_type_bl_, true);

    // Reset the motion type selection
    motion_type_bl_.combo_box->setCurrentText("");

    // Hide the arrow marker
    this->hide_im(this->arrow_marker_name_);
}

void CcaInteractiveGoals::create_button_(QPushButton *&button, const QString &text, QVBoxLayout *layout)
{
    button = new QPushButton(text);
    button->setEnabled(false);
    layout->addWidget(button);
}

QHBoxLayout *CcaInteractiveGoals::create_combo_box_layout_(const QString &label, QComboBox *&combo_box,
                                                           const QStringList &items)
{
    combo_box = new QComboBox;
    combo_box->addItems(items);

    // Add to layout
    auto *layout = new QHBoxLayout;
    layout->addWidget(new QLabel(label));
    layout->addWidget(combo_box);
    return layout;
}
QHBoxLayout *CcaInteractiveGoals::create_combo_box_layout_(const QString &label, QComboBoxAndLabel &bl,
                                                           const QStringList &items)
{
    bl.label = new QLabel(label);
    bl.combo_box = new QComboBox;
    bl.combo_box->addItems(items);

    // Add to layout
    auto *layout = new QHBoxLayout;
    layout->addWidget(bl.label);
    layout->addWidget(bl.combo_box);
    return layout;
}

QHBoxLayout *CcaInteractiveGoals::create_line_edit_layout_(const QString &label, QLineEditAndLabel &ll)
{
    ll.label = new QLabel(label);
    ll.line_edit = new QLineEdit;

    // Add to layout
    auto *layout = new QHBoxLayout;
    layout->addWidget(ll.label);
    layout->addWidget(ll.line_edit);
    return layout;
}

void CcaInteractiveGoals::connect_signals_()
{
    connect(mode_bl_.combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(mode_selected_()));
    connect(motion_type_bl_.combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(motion_type_selected_()));
    connect(goal_bl_.combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(goal_selected_(int)));
    connect(axis_bl_.combo_box, SIGNAL(currentTextChanged(QString)), this, SLOT(axis_option_selected_(QString)));
    connect(pitch_bl_.combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(pitch_selected_(int)));
    connect(plan_viz_button_, SIGNAL(clicked()), this, SLOT(plan_button_clicked_()));
    connect(plan_viz_exe_button_, SIGNAL(clicked()), this, SLOT(plan_exe_button_clicked_()));
    connect(plan_exe_button_, SIGNAL(clicked()), this, SLOT(exe_button_clicked_()));
    connect(stop_button_, SIGNAL(clicked()), this, SLOT(cancel_exe_button_clicked_()));
    connect(apply_button_, SIGNAL(clicked()), this, SLOT(apply_settings_clicked_()));
}

template <typename ValueType>
QStringList CcaInteractiveGoals::get_map_keys_(const std::map<QString, ValueType> &map, bool prepend_empty)
{
    QStringList keys;
    // If asked, add an empty string in the beginning
    if (prepend_empty)
    {
        keys.append("");
    }

    for (const auto &pair : map)
    {
        keys.append(pair.first); // Append the key to the list
    }
    return keys;
}

void CcaInteractiveGoals::onInitialize()
{
    // Do ROS client initializations in this function
    // Initialize the CCA Ros action client to be able to send planning requests
    ccaRosActionClient = std::make_shared<cca_ros_action::CcaRosActionClient>();

    // Set up timer for spinning the node
    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, &CcaInteractiveGoals::spin);
    spin_timer_->start(10); // Spin every 10ms
}
void CcaInteractiveGoals::mode_selected_()
{
    // Disable all execution buttons
    set_execute_buttons_enabled_(false);

    // Hide the arrow marker
    this->hide_im(this->arrow_marker_name_);

    // Check if a mode is selected
    bool mode_selected = mode_bl_.combo_box->currentIndex() != -1;
    if (!mode_selected)
        return;

    // Get the current selected mode
    QString selected_mode = mode_bl_.combo_box->currentText();

    // Reset all UI elements first
    hide_all_controls_();

    // Configure UI based on the selected mode
    if (selected_mode == "Affordance")
    {
        // Show motion type controls
        set_combo_box_controls_(motion_type_bl_, true);
        motion_type_bl_.combo_box->setCurrentText("");
    }
    else if (selected_mode == "EE Orientation Only")
    {
        // Show axis controls
        set_combo_box_controls_(axis_bl_, true);
        axis_bl_.combo_box->setCurrentText("");
    }
}

void CcaInteractiveGoals::motion_type_selected_()
{
    // Disable execution buttons
    set_execute_buttons_enabled_(false);

    // Hide value and pitch controls initially
    set_line_edit_controls_(value_ll_, false);
    set_combo_box_controls_(pitch_bl_, false);
    set_line_edit_controls_(pitch_value_ll_, false);

    // Reset pitch combo box
    pitch_bl_.combo_box->setCurrentIndex(0);

    QString motion_type = motion_type_bl_.combo_box->currentText();

    // Handle interactive marker visibility
    if (motion_type == "Translation" || motion_type == "Rotation" || motion_type == "Screw")
    {
        // Enable arrow marker controls
        interactive_marker_manager::ImControlEnableInfo arrow_enable_info;
        arrow_enable_info.marker_name = this->arrow_marker_name_;
        arrow_enable_info.enable = interactive_marker_manager::ImControlEnable::ALL;
        arrow_enable_info.reset = false;
        this->enable_im_controls(arrow_enable_info);
    }
    else
    {
        this->hide_im(this->arrow_marker_name_);
    }

    // Show pitch controls for Screw motion type
    if (motion_type == "Screw")
    {
        set_combo_box_controls_(pitch_bl_, true);
    }

    // Configure goal controls based on motion type
    setup_goal_controls_for_motion_type_(motion_type);
}

void CcaInteractiveGoals::goal_selected_(int index)
{
    // Show value controls for custom value (index 1)
    if (index == 1)
    {
        set_line_edit_controls_(value_ll_, true);
        setup_value_label_text_();
    }
    else
    {
        set_line_edit_controls_(value_ll_, false);
    }

    // Enable execution buttons when valid selections are made
    update_execution_buttons_state_(index);
}

void CcaInteractiveGoals::pitch_selected_(int index)
{
    // Show pitch value controls for custom pitch (index 1)
    set_line_edit_controls_(pitch_value_ll_, index == 1);

    // Enable execution buttons when valid selections are made
    update_execution_buttons_state_(goal_bl_.combo_box->currentIndex());
}

void CcaInteractiveGoals::axis_option_selected_(QString axis)
{
    if (axis.isEmpty())
        return;

    // Setup goal controls for rotation
    set_combo_box_controls_(goal_bl_, true);
    goal_bl_.combo_box->setCurrentIndex(0);
    goal_bl_.label->setText("Goal Angle(radians)");
    goal_bl_.combo_box->clear();
    goal_bl_.combo_box->addItems(ROTATION_GOALS_);

    // Draw the appropriate interactive marker
    this->draw_ee_or_control_im(axis.toStdString());
}

void CcaInteractiveGoals::set_execute_buttons_enabled_(bool enabled)
{
    plan_viz_button_->setEnabled(enabled);
    plan_viz_exe_button_->setEnabled(enabled);
    plan_exe_button_->setEnabled(enabled);
    stop_button_->setEnabled(enabled);
}

void CcaInteractiveGoals::hide_all_controls_()
{
    // Hide all UI control groups
    set_combo_box_controls_(axis_bl_, false);
    set_combo_box_controls_(motion_type_bl_, false);
    set_combo_box_controls_(goal_bl_, false);
    set_line_edit_controls_(value_ll_, false);
    set_combo_box_controls_(pitch_bl_, false);
    set_line_edit_controls_(pitch_value_ll_, false);
}

void CcaInteractiveGoals::set_combo_box_controls_(const QComboBoxAndLabel &controls, bool visible)
{
    if (controls.label)
    {
        controls.label->setVisible(visible);
    }
    if (controls.combo_box)
    {
        controls.combo_box->setEnabled(visible);
        controls.combo_box->setVisible(visible);
    }
}

void CcaInteractiveGoals::set_line_edit_controls_(const QLineEditAndLabel &controls, bool visible)
{
    if (controls.label)
    {
        controls.label->setVisible(visible);
    }
    if (controls.line_edit)
    {
        controls.line_edit->setEnabled(visible);
        controls.line_edit->setVisible(visible);
    }
}

void CcaInteractiveGoals::setup_goal_controls_for_motion_type_(const QString &motion_type)
{
    // Enable goal boxes
    set_combo_box_controls_(goal_bl_, true);
    goal_bl_.combo_box->setCurrentIndex(0);

    if (motion_type == "Translation")
    {
        goal_bl_.label->setText("Goal Distance(meters)");
        goal_bl_.combo_box->clear();
        goal_bl_.combo_box->addItems(TRANSLATION_GOALS_);
    }
    else if (motion_type == "Rotation" || motion_type == "Screw")
    {
        goal_bl_.label->setText("Goal Angle(radians)");
        goal_bl_.combo_box->clear();
        goal_bl_.combo_box->addItems(ROTATION_GOALS_);
    }
}

void CcaInteractiveGoals::setup_value_label_text_()
{
    QString motion_type = motion_type_bl_.combo_box->currentText();
    QString mode = mode_bl_.combo_box->currentText();

    if (motion_type == "Translation" && mode != "EE Orientation Only")
    {
        value_ll_.label->setText("Meters");
    }
    else if (motion_type == "Rotation" || motion_type == "Screw" || mode == "EE Orientation Only")
    {
        value_ll_.label->setText("Radians");
    }
}

void CcaInteractiveGoals::update_execution_buttons_state_(int goal_index)
{
    bool enable_buttons = false;

    if (goal_index != 0)
    {
        QString motion_type = motion_type_bl_.combo_box->currentText();

        if (motion_type != "Screw" || pitch_bl_.combo_box->currentIndex() != 0)
        {
            enable_buttons = true;
        }
    }

    set_execute_buttons_enabled_(enable_buttons);
}

void CcaInteractiveGoals::plan_button_clicked_()
{
    // Enable stop button
    stop_button_->setEnabled(true);

    // Fill out planning request and send it to the server
    auto req = buildPlanningRequest();
    req.visualize_trajectory = true;
    req.execute_trajectory = false;
    ccaRosActionClient->send_goal(req);
}

void CcaInteractiveGoals::plan_exe_button_clicked_()
{
    // Enable stop button
    stop_button_->setEnabled(true);

    // Fill out planning request and send it to the server
    auto req = buildPlanningRequest();
    req.visualize_trajectory = true;
    req.execute_trajectory = true;
    ccaRosActionClient->send_goal(req);
}

void CcaInteractiveGoals::exe_button_clicked_()
{
    // Enable stop button
    stop_button_->setEnabled(true);

    // Fill out planning request and send it to the server
    auto req = buildPlanningRequest();
    req.visualize_trajectory = false;
    req.execute_trajectory = true;
    ccaRosActionClient->send_goal(req);
}

void CcaInteractiveGoals::cancel_exe_button_clicked_() { ccaRosActionClient->cancel_goal(); }

void CcaInteractiveGoals::apply_settings_clicked_()
{
    AdvancedSettings advanced_settings;

    // Use the advanced_settings_widgets_ struct to access the widgets
    if (advanced_settings_widgets_.accuracy->text().toFloat() != 0)
    {
        advanced_settings.planner_config.accuracy = advanced_settings_widgets_.accuracy->text().toFloat();
    }

    if (advanced_settings_widgets_.closure_angle->text().toFloat() != 0)
    {
        advanced_settings.planner_config.closure_err_threshold_ang =
            advanced_settings_widgets_.closure_angle->text().toFloat();
    }

    if (advanced_settings_widgets_.closure_linear->text().toFloat() != 0)
    {
        advanced_settings.planner_config.closure_err_threshold_lin =
            advanced_settings_widgets_.closure_linear->text().toFloat();
    }

    if (advanced_settings_widgets_.ik_iterations->text().toInt() != 0)
    {
        advanced_settings.planner_config.ik_max_itr = advanced_settings_widgets_.ik_iterations->text().toInt();
    }

    if (advanced_settings_widgets_.trajectory_density->text().toInt() != 0)
    {
        advanced_settings.task_description.trajectory_density =
            advanced_settings_widgets_.trajectory_density->text().toInt();
    }

    // Use the screw_order_combo_ from the struct
    if (advanced_settings_widgets_.vir_screw_order->currentIndex() != 0)
    {
        advanced_settings.task_description.vir_screw_order =
            vir_screw_order_map_.at(advanced_settings_widgets_.vir_screw_order->currentText());
    }

    // Use the cca_type_combo_ from the struct if needed
    if (advanced_settings_widgets_.cca_type->currentIndex() != 0)
    {
        advanced_settings.cca_type = cca_type_map_.at(advanced_settings_widgets_.cca_type->currentText());
    }

    advanced_settings_ = advanced_settings;
    new_settings_applied_ = true;

    RCLCPP_INFO(this->get_logger(), "New Advanced Settings Applied");
}

cca_ros::PlanningRequest CcaInteractiveGoals::buildPlanningRequest()
{

    cca_ros::PlanningRequest req;

    // Set planning request start state for testing purposes -- temporary, to be removed
    const Eigen::VectorXd READY_CONFIG =
        (Eigen::VectorXd(6) << -0.00015592575073242188, -0.8980185389518738, 1.8094338178634644, 0.000377655029296875,
         -0.8991076946258545, 0.0015475749969482422)
            .finished();
    req.start_state.robot = READY_CONFIG;

    // Deduce planning type
    const auto planning_type = planning_type_map_.at(mode_bl_.combo_box->currentText());
    req.task_description = cc_affordance_planner::TaskDescription(planning_type);

    // If affordance, fill out relevant screw-type (and pitch) info
    if (planning_type == cc_affordance_planner::PlanningType::AFFORDANCE)
    {
        const auto motion_type = motion_type_map_.at(motion_type_bl_.combo_box->currentText());
        req.task_description.affordance_info.type = motion_type;

        if (motion_type == affordance_util::ScrewType::SCREW)
        {
            // Fill out the pitch from the dropdown menu
            if (pitch_bl_.combo_box->currentText() != "Manual Input")
            {
                req.task_description.affordance_info.pitch =
                    std::stof(pitch_bl_.combo_box->currentText().toStdString());
            }
            else // Manual input
            {
                try
                {
                    req.task_description.affordance_info.pitch =
                        std::stof(pitch_value_ll_.line_edit->text().toStdString());
                }
                catch (int)
                {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Screw Pitch could not be converted to type float. Please check entry and try again");
                }
            }
        }
    }

    // Get affordance goal
    req.task_description.goal.affordance = get_affordance_goal_();

    // Get affordance pose
    const auto screw_info = this->get_arrow_pose(mode_bl_.combo_box->currentText().toStdString(),
                                                 axis_bl_.combo_box->currentText().toStdString());
    req.task_description.affordance_info.axis = screw_info.axis;
    req.task_description.affordance_info.location = screw_info.location;

    // Extract task-specific settings from advanced settings
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

double CcaInteractiveGoals::get_affordance_goal_()
{
    double goal;

    if ((motion_type_bl_.combo_box->currentText() == "Rotation" ||
         motion_type_bl_.combo_box->currentText() == "Screw" ||
         mode_bl_.combo_box->currentText() == "EE Orientation Only") &&
        (goal_bl_.combo_box->currentText() != "Manual Input"))
    {
        goal = M_PI * (goal_bl_.combo_box->currentIndex() - 1) / 4.0; // multiple of pi/4
    }
    else if (motion_type_bl_.combo_box->currentText() == "Translation" &&
             goal_bl_.combo_box->currentText() != "Manual Input")
    {
        goal = std::stof(goal_bl_.combo_box->currentText().toStdString());
    }
    else
    { // Manual Input
        try
        {
            goal = std::stof(value_ll_.line_edit->text().toStdString());
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

void CcaInteractiveGoals::spin() { rclcpp::spin_some(this->get_node_base_interface()); }

} // namespace cca_interactive_goals

// Export the CcaInteractiveGoals class as a plugin for rviz_common::Panel
PLUGINLIB_EXPORT_CLASS(cca_interactive_goals::CcaInteractiveGoals, rviz_common::Panel)
