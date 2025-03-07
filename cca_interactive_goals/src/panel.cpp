#include "cca_interactive_goals/panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QString>
#include <QTimer>
#include <QFormLayout>
#include <vector>
#include <string>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace cca_interactive_goals
{

CcaInteractiveGoals::CcaInteractiveGoals(QWidget* parent)
  : rviz_common::Panel(parent), rclcpp::Node("cca_interactive_goals")
{
  QTabWidget* tabWidget = new QTabWidget(this);

  // CCA_IG Tab
  QWidget* ccaIgTabWidget = new QWidget();
  QVBoxLayout* mainLayout = new QVBoxLayout;

  // Mode selection dropdown

  QHBoxLayout* mode_layout = new QHBoxLayout;
  QLabel* mode_label = new QLabel("Select Mode:");
  mode_combo_box_ = new QComboBox;
  mode_combo_box_->addItems({ "Affordance Planning", "Cartesian Goal Planning",
                              "In-Place End Effector Orientation Control", "Approach Motion Task" });
  mode_layout->addWidget(mode_label);
  mode_layout->addWidget(mode_combo_box_);
  mainLayout->addLayout(mode_layout);

  // Dynamic content layout

  QVBoxLayout* dynamicContentLayout = new QVBoxLayout;
  dynamicContentLayout->setSpacing(10);  // Set spacing between dynamic boxes

  // Motion type dropdown

  QHBoxLayout* motion_type_layout = new QHBoxLayout;
  motion_type_label_ = new QLabel("Motion Type:");
  motion_type_combo_box_ = new QComboBox;
  motion_type_combo_box_->addItems({ "", "Translation", "Rotation", "Screw Motion" });
  motion_type_layout->addWidget(motion_type_label_);
  motion_type_layout->addWidget(motion_type_combo_box_);
  dynamicContentLayout->addLayout(motion_type_layout);

  // Axis Selection Dropdown

  QHBoxLayout* axis_layout = new QHBoxLayout;
  axis_label_ = new QLabel("Axis:");
  axis_combo_box_ = new QComboBox;
  axis_combo_box_->addItems({ "", "Manual Input", "x", "y", "z", "-x", "-y", "-z" });
  axis_layout->addWidget(axis_label_);
  axis_layout->addWidget(axis_combo_box_);
  dynamicContentLayout->addLayout(axis_layout);

  // Screw placement button

  conf_place_button_ = new QPushButton("Confirm Screw Placement");
  dynamicContentLayout->addWidget(conf_place_button_);
  conf_place_button_->setEnabled(false);

  // Frame placement button

  frame_place_button_ = new QPushButton("Confirm Frame Placement");
  dynamicContentLayout->addWidget(frame_place_button_);
  frame_place_button_->setEnabled(false);

  // Pitch Selection Dropdown

  QHBoxLayout* pitch_layout = new QHBoxLayout;
  pitch_label_ = new QLabel("Pitch(meters/radian):");
  pitch_combo_box_ = new QComboBox;
  pitch_combo_box_->addItems({ "", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5" });
  pitch_layout->addWidget(pitch_label_);
  pitch_layout->addWidget(pitch_combo_box_);
  dynamicContentLayout->addLayout(pitch_layout);

  // Pitch input

  QHBoxLayout* pitch_value_layout = new QHBoxLayout;
  pitch_value_label_ = new QLabel("Pitch Value(meters/radian):");
  pitch_value_label_->setObjectName("Pitch Value Label");
  pitch_value_input_ = new QLineEdit;
  pitch_value_layout->addWidget(pitch_value_label_);
  pitch_value_layout->addWidget(pitch_value_input_);
  dynamicContentLayout->addLayout(pitch_value_layout);

  // Goal Selection Dropdown

  QHBoxLayout* goal_layout = new QHBoxLayout;
  goal_label_ = new QLabel("Goal:");
  goal_combo_box_ = new QComboBox;
  goal_combo_box_->addItems(
      { "", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6", "0.7", "0.8", "0.9", "1.0" });
  goal_layout->addWidget(goal_label_);
  goal_layout->addWidget(goal_combo_box_);
  dynamicContentLayout->addLayout(goal_layout);

  // Goal input

  QHBoxLayout* value_layout = new QHBoxLayout;
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

  QVBoxLayout* buttonLayout = new QVBoxLayout;
  plan_viz_button_ = new QPushButton("Plan + Visualize");
  buttonLayout->addWidget(plan_viz_button_);
  plan_viz_button_->setEnabled(false);
  plan_viz_exe_button_ = new QPushButton("Plan, Visualize + Execute");
  buttonLayout->addWidget(plan_viz_exe_button_);
  plan_viz_exe_button_->setEnabled(false);
  plan_exe_button_ = new QPushButton("Plan + Execute");
  buttonLayout->addWidget(plan_exe_button_);
  plan_exe_button_->setEnabled(false);
  stop_button_ = new QPushButton("STOP");
  buttonLayout->addWidget(stop_button_);
  stop_button_->setEnabled(false);

  mainLayout->addLayout(buttonLayout);

  ccaIgTabWidget->setLayout(mainLayout);

  // Advanced Settings Tab
  QWidget* advancedSettingsTabWidget = new QWidget();
  QVBoxLayout* advancedLayout = new QVBoxLayout;
  QFormLayout* form_layout = new QFormLayout;

  // Text input fields
  accuracy_ = new QLineEdit();
  closure_angle_ = new QLineEdit();
  closure_linear_ = new QLineEdit();
  ik_iterations_ = new QLineEdit();
  trajectory_density_ = new QLineEdit();

  // Dropdown menus
  screw_order_combo_ = new QComboBox();
  QStringList screw_options = { "XYZ", "YZX", "ZXY", "XY", "YZ", "ZX" };
  screw_order_combo_->addItems(screw_options);

  cca_type_combo_ = new QComboBox();
  cca_type_combo_->addItems({ "Affordance Control", "Affordance and EE Control" });

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

  QVBoxLayout* button_layout = new QVBoxLayout();
  button_layout->addWidget(apply_button_);

  advancedLayout->addLayout(button_layout);  //

  advancedSettingsTabWidget->setLayout(advancedLayout);

  tabWidget->addTab(ccaIgTabWidget, "CCA_IG");
  tabWidget->addTab(advancedSettingsTabWidget, "Advanced Settings");

  QVBoxLayout* panelLayout = new QVBoxLayout;
  panelLayout->addWidget(tabWidget);
  setLayout(panelLayout);

  setFixedSize(400, 475);  // Set fixed size for the panel

  // Linking of the callback functions

  connect(mode_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(modeSelected(int)));
  connect(motion_type_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(motionTypeSelected(int)));
  connect(goal_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(goalSelected(int)));
  connect(axis_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(axisOptionSelected(int)));
  connect(pitch_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(pitchSelected(int)));
  connect(frame_place_button_, SIGNAL(clicked()), this, SLOT(framePlaceButtonClicked()));
  connect(plan_viz_button_, SIGNAL(clicked()), this, SLOT(planVizClicked()));
  connect(plan_viz_exe_button_, SIGNAL(clicked()), this, SLOT(planVizExeClicked()));
  connect(plan_exe_button_, SIGNAL(clicked()), this, SLOT(planExeClicked()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stopClicked()));
  connect(conf_place_button_, SIGNAL(clicked()), this, SLOT(confirmPlaceClicked()));

  connect(apply_button_, SIGNAL(clicked()), this, SLOT(applySettingsClicked()));

  updateUIState();

  // Interactive marker initialization
}

void CcaInteractiveGoals::onInitialize()
{
  // Anything related to spinning up the node or pub/subs/servers must be done in here.
  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "interactive_goals", this->get_node_base_interface(), this->get_node_clock_interface(),
      this->get_node_logging_interface(), this->get_node_topics_interface(), this->get_node_services_interface());

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  screw_info_publisher_ = this->create_publisher<interactive_goal_interfaces::msg::ScrewInfo>("/screw_info", 10);

  button_press_publisher_ = this->create_publisher<interactive_goal_interfaces::msg::ButtonPress>("/button_press", 10);

  settings_publisher_ = this->create_publisher<interactive_goal_interfaces::msg::AdvancedSettings>("/settings", 10);

  // Initialize CCA action client
  cca_action_client_ =
      rclcpp_action::create_client<CcaRosAction>(this, cca_as_name_);

  // Create and hide interactive markers
  createArrowInteractiveMarker();
  createInvisibleInteractiveMarker();
  server_->applyChanges();

  disableInteractiveMarkerControls("arrow_marker");
  disableInteractiveMarkerControls("approach_frame");

  // Set up timer for spinning the node
  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &CcaInteractiveGoals::spin);
  spin_timer_->start(10);  // Spin every 10ms
}

void CcaInteractiveGoals::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void CcaInteractiveGoals::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void CcaInteractiveGoals::planVizClicked()
{
  // Handle plan viz button click
  interactive_goal_interfaces::msg::ButtonPress buttons;
  buttons.execute = false;
  buttons.plan = true;
  buttons.visualize = true;
  buttons.stop = false;
  screwInfoBuilder();
  button_press_publisher_->publish(buttons);
  stop_button_->setEnabled(true);

  RCLCPP_INFO(this->get_logger(), "planViz button pressed");
}

void CcaInteractiveGoals::planVizExeClicked()
{
  // Handle plan viz execute button click
  interactive_goal_interfaces::msg::ButtonPress buttons;
  buttons.execute = true;
  buttons.plan = true;
  buttons.visualize = true;
  buttons.stop = false;
  screwInfoBuilder();
  button_press_publisher_->publish(buttons);
  stop_button_->setEnabled(true);

  RCLCPP_INFO(this->get_logger(), "planVizExe button pressed");
}

void CcaInteractiveGoals::planExeClicked()
{
  // Handle plan execute button click
  interactive_goal_interfaces::msg::ScrewInfo screw;
  interactive_goal_interfaces::msg::ButtonPress buttons;
  buttons.execute = false;
  buttons.plan = true;
  buttons.visualize = false;
  buttons.stop = false;
  screwInfoBuilder();
  button_press_publisher_->publish(buttons);

  stop_button_->setEnabled(true);
  RCLCPP_INFO(this->get_logger(), "planExe button pressed");
}

void CcaInteractiveGoals::stopClicked()
{
  // Handle stop button click
  interactive_goal_interfaces::msg::ButtonPress stop_command;
  stop_command.execute = false;
  stop_command.plan = false;
  stop_command.visualize = false;
  stop_command.stop = true;
}

void CcaInteractiveGoals::screwInfoBuilder()
{
  // interactive_goal_interfaces::msg::ScrewInfo plan_description;

  if (mode_combo_box_->currentIndex() == 0) // Affordance Planning
  {
    // plan_description.category = 0;
    req_.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::AFFORDANCE);

  }
  else if (mode_combo_box_->currentIndex() == 1)
  {
    // plan_description.category = 1;
    req_.task_description =
    cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::CARTESIAN_GOAL);
  }
  else if (mode_combo_box_->currentIndex() == 2)
  {
    // plan_description.category = 2;
   req_.task_description =
    cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY);
  }
  else if (mode_combo_box_->currentIndex() == 3)
  {
    // plan_description.category = 3;
  req_.task_description = cc_affordance_planner::TaskDescription(cc_affordance_planner::PlanningType::APPROACH);
  }

  // Determine motion type for affordance and approach motion
  if (mode_combo_box_->currentIndex() == 0 || mode_combo_box_->currentIndex() == 3)
  {
    if (motion_type_combo_box_->currentIndex() == 1)
    {
      // plan_description.type = 0;  // Translation
      req_.task_description.affordance_info.type = affordance_util::ScrewType::TRANSLATION;

    }
    else if (motion_type_combo_box_->currentIndex() == 2)
    {
      // plan_description.type = 1;  // Rotation
      req_.task_description.affordance_info.type = affordance_util::ScrewType::ROTATION;
    }
    else if (motion_type_combo_box_->currentIndex() == 3)
    {
      // plan_description.type = 2;  // Screw Motion
      req_.task_description.affordance_info.type = affordance_util::ScrewType::SCREW;

      // Determine pitch value
      if (pitch_combo_box_->currentIndex() != 1)
      {
        // plan_description.pitch = std::stof(pitch_combo_box_->currentText().toStdString());
      req_.task_description.affordance_info.pitch = std::stof(pitch_combo_box_->currentText().toStdString());

      }
      else
      {
        try
        {
          // plan_description.pitch = std::stof(pitch_value_input_->text().toStdString());
req_.task_description.affordance_info.pitch = std::stof(pitch_value_input_->text().toStdString());
        }
        catch (int)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "Screw Pitch could not be converted to type float. Please check entry and try again");
        }
      }
    }
  }
  // Determine goal value for all but CGP
  if (mode_combo_box_->currentIndex() != 1)
  {
    if (goal_combo_box_->currentIndex() != 1)
    {
      // if (plan_description.type == 0)
	if (motion_type_combo_box_->currentIndex() == 1)// Translation
      {
        // plan_description.goal = std::stof(goal_combo_box_->currentText().toStdString());
	req_.task_description.goal.affordance= std::stof(goal_combo_box_->currentText().toStdString());

        RCLCPP_INFO_STREAM(this->get_logger(), "PLAN GOAL IS"<<plan_description.goal);
      }
      else // Rotation or Screw
      {
        // plan_description.goal = M_PI * (goal_combo_box_->currentIndex() - 1) / 4.0; // multiple of pi/4
        req_.task_description.goal.affordance= M_PI * (goal_combo_box_->currentIndex() - 1) / 4.0; // multiple of pi/4
      }
    }
    else 
    {
      try
      {
        // plan_description.goal = std::stof(value_input_->text().toStdString());
	req_.task_description.goal.affordance= std::stof(value_input_->text().toStdString());

      }
      catch (int)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal could not be converted to type float. Please check entry and try again");
      }
    }
  }

  // screw_info_publisher_->publish(plan_description);
}

void CcaInteractiveGoals::confirmPlaceClicked()
{
  value_input_->setVisible(false);
  value_input_->setEnabled(false);
  value_label_->setVisible(false);

  if (mode_combo_box_->currentText() == "Affordance Planning")
  {
    // Handle confirm screw placement button click
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
      std::vector<std::string> pi_fractions = {
        "π/4", "π/2", "3π/4", "π", "5π/4", "3π/2", "7π/4", "2π", "9π/4", "5π/2"
      };
      for (int i = 2; i <= 11; i++)
      {
        goal_combo_box_->setItemText(i, pi_fractions[i - 2].c_str());
      }
    }
  }
  else
  {
    frame_place_button_->setVisible(true);
    frame_place_button_->setEnabled(true);
    enableInteractiveMarkerControls("approach_frame");
  }
}

void CcaInteractiveGoals::framePlaceButtonClicked()
{
  if (mode_combo_box_->currentText() == "Approach Motion Task")
  {
    // Handle confirm frame place button click

    // Handle confirm screw placement button click
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
      std::vector<std::string> pi_fractions = {
        "π/4", "π/2", "3π/4", "π", "5π/4", "3π/2", "7π/4", "2π", "9π/4", "5π/2"
      };
      for (int i = 2; i <= 11; i++)
      {
        goal_combo_box_->setItemText(i, pi_fractions[i - 2].c_str());
      }
    }
  }
  else
  {
    plan_exe_button_->setEnabled(true);
    plan_viz_button_->setEnabled(true);
    plan_viz_exe_button_->setEnabled(true);
  }
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

  disableInteractiveMarkerControls("arrow_marker");
  disableInteractiveMarkerControls("approach_frame");

  if (mode_selected)
  {
    if (mode_combo_box_->currentText() == "Affordance Planning" ||
        mode_combo_box_->currentText() == "Approach Motion Task")
    {
      // TODO: set all others to not visible

      axis_combo_box_->setEnabled(false);
      axis_combo_box_->setVisible(false);
      axis_label_->setVisible(false);
      goal_label_->setVisible(false);
      goal_combo_box_->setEnabled(false);
      goal_combo_box_->setVisible(false);
      value_input_->setVisible(false);
      value_input_->setEnabled(false);
      value_label_->setVisible(false);
      frame_place_button_->setVisible(false);
      pitch_label_->setVisible(false);
      pitch_combo_box_->setVisible(false);
      pitch_value_input_->setVisible(false);
      pitch_value_label_->setVisible(false);
      frame_place_button_->setVisible(false);
      frame_place_button_->setEnabled(false);

      // Set necessary widgets to visible
      motion_type_label_->setVisible(true);
      motion_type_combo_box_->setEnabled(true);
      motion_type_combo_box_->setVisible(true);
      motion_type_combo_box_->setCurrentText("");
      conf_place_button_->setEnabled(false);
      conf_place_button_->setVisible(true);
    }
    else if (mode_combo_box_->currentText() == "Cartesian Goal Planning")
    {
      axis_combo_box_->setEnabled(false);
      axis_combo_box_->setVisible(false);
      axis_label_->setVisible(false);
      motion_type_label_->setVisible(false);
      motion_type_combo_box_->setEnabled(false);
      motion_type_combo_box_->setVisible(false);
      conf_place_button_->setEnabled(false);
      conf_place_button_->setVisible(false);
      goal_label_->setVisible(false);
      goal_combo_box_->setEnabled(false);
      goal_combo_box_->setVisible(false);
      value_input_->setVisible(false);
      value_input_->setEnabled(false);
      value_label_->setVisible(false);
      frame_place_button_->setVisible(false);
      pitch_label_->setVisible(false);
      pitch_combo_box_->setVisible(false);
      pitch_value_input_->setVisible(false);
      pitch_value_label_->setVisible(false);

      // Set necessary widgets to visible

      frame_place_button_->setVisible(true);
      frame_place_button_->setEnabled(true);
      enableInteractiveMarkerControls("approach_frame");
    }
    else if (mode_combo_box_->currentText() == "In-Place End Effector Orientation Control")
    {
      motion_type_label_->setVisible(false);
      motion_type_combo_box_->setEnabled(false);
      motion_type_combo_box_->setVisible(false);
      conf_place_button_->setEnabled(false);
      conf_place_button_->setVisible(false);
      goal_label_->setVisible(false);
      goal_combo_box_->setEnabled(false);
      goal_combo_box_->setVisible(false);
      value_input_->setVisible(false);
      value_input_->setEnabled(false);
      value_label_->setVisible(false);
      frame_place_button_->setVisible(false);
      pitch_label_->setVisible(false);
      pitch_combo_box_->setVisible(false);
      pitch_value_input_->setVisible(false);
      pitch_value_label_->setVisible(false);
      frame_place_button_->setVisible(false);
      frame_place_button_->setEnabled(false);

      // Set necessary widgets to visible

      axis_combo_box_->setEnabled(true);
      axis_combo_box_->setVisible(true);
      axis_label_->setVisible(true);
      axis_combo_box_->setCurrentIndex(0);
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
  frame_place_button_->setVisible(false);
  frame_place_button_->setEnabled(false);
  if (motion_type_combo_box_->currentText() == "Translation" || motion_type_combo_box_->currentText() == "Rotation" ||
      motion_type_combo_box_->currentText() == "Screw Motion")
  {
    conf_place_button_->setEnabled(true);
    conf_place_button_->setVisible(true);
    enableInteractiveMarkerControls("arrow_marker");
  }
  else
  {
    conf_place_button_->setEnabled(false);
    conf_place_button_->setVisible(true);
    disableInteractiveMarkerControls("arrow_marker");
    disableInteractiveMarkerControls("approach_frame");
  }
  if (motion_type_combo_box_->currentText() == "Screw Motion")
  {
    pitch_label_->setVisible(true);
    pitch_combo_box_->setVisible(true);
    pitch_value_input_->setVisible(false);
    pitch_value_label_->setVisible(false);
  }
  goal_label_->setVisible(false);
  goal_combo_box_->setEnabled(false);
  goal_combo_box_->setVisible(false);
  if (index != 0)
  {
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
  if (index != 0 && (motion_type_combo_box_->currentText() != "Screw Motion" || pitch_combo_box_->currentIndex() != 0))
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

void CcaInteractiveGoals::axisOptionSelected(int index)
{
  if (index != 0)
  {
    goal_label_->setVisible(true);
    goal_combo_box_->setEnabled(true);
    goal_combo_box_->setVisible(true);
    goal_combo_box_->setCurrentIndex(0);
    goal_label_->setText("Goal Angle(radians)");
    std::vector<std::string> pi_fractions = { "π/4", "π/2", "3π/4", "π", "5π/4", "3π/2", "7π/4", "2π", "9π/4", "5π/2" };
    for (int i = 2; i <= 11; i++)
    {
      goal_combo_box_->setItemText(i, pi_fractions[i - 2].c_str());
    }
    enableInteractiveMarkerControls("arrow_marker");
  }

  visualization_msgs::msg::InteractiveMarker int_marker;
  server_->get("arrow_marker", int_marker);
  visualization_msgs::msg::InteractiveMarkerControl arrow;
  for (auto& control : int_marker.controls)
  {
    if (!control.markers.empty())
    {
      for (auto& marker : control.markers)
      {
        if (marker.id = 8)
        {
          RCLCPP_INFO(this->get_logger(), "Arrow Located");
          arrow = control;
        }
      }
    }
  }
  if (index == 1)
  {
  }
  else if (index == 2)
  {
    arrow.orientation.w = 1;
    arrow.orientation.x = 0;
    arrow.orientation.y = 0;
    arrow.orientation.z = 0;

    server_->insert(int_marker);
    server_->applyChanges();
  }
  else if (index == 3)
  {
    arrow.orientation.w = 0.707;
    arrow.orientation.x = 0;
    arrow.orientation.y = 0;
    arrow.orientation.z = 0.707;

    server_->insert(int_marker);
    server_->applyChanges();
  }
  else if (index == 4)
  {
    arrow.orientation.w = 0.707;
    arrow.orientation.x = 0;
    arrow.orientation.y = -0.707;
    arrow.orientation.z = 0;

    server_->insert(int_marker);
    server_->applyChanges();
  }
  else if (index == 5)
  {
    arrow.orientation.w = 0;
    arrow.orientation.x = 0;
    arrow.orientation.y = 0;
    arrow.orientation.z = 1;

    server_->insert(int_marker);
    server_->applyChanges();
  }
  else if (index == 6)
  {
    arrow.orientation.w = 0.707;
    arrow.orientation.x = 0;
    arrow.orientation.y = 0;
    arrow.orientation.z = -0.707;

    server_->insert(int_marker);
    server_->applyChanges();
  }
  else if (index == 7)
  {
    arrow.orientation.w = 0.707;
    arrow.orientation.x = 0;
    arrow.orientation.y = 0.707;
    arrow.orientation.z = 0;

    server_->insert(int_marker);
    server_->applyChanges();
  }
  enableInteractiveMarkerControls("arrow_marker");
}

void CcaInteractiveGoals::applySettingsClicked()
{
  interactive_goal_interfaces::msg::AdvancedSettings settings;

  bool conversion_ok = true;

  // Convert float values with error checking
  settings.accuracy = accuracy_->text().toFloat(&conversion_ok);
  if (!conversion_ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid float value in Accuracy field");
    settings.change_accuracy = false;
  }
  else
  {
    settings.change_accuracy = true;
  }

  settings.closure_err_threshold_ang = closure_angle_->text().toFloat(&conversion_ok);
  if (!conversion_ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid float value in Closure Angle field");
    settings.change_close_ang = false;
  }
  else
  {
    settings.change_close_ang = true;
  }

  settings.closure_err_threshold_lin = closure_linear_->text().toFloat(&conversion_ok);
  if (!conversion_ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid float value in Closure Linear field");
    settings.change_close_lin = false;
  }
  else
  {
    settings.change_close_lin = true;
  }

  // Convert integer values with error checking
  settings.ik_max_itr = ik_iterations_->text().toInt(&conversion_ok);
  if (!conversion_ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid integer value in IK Iterations field");
    settings.change_ik = false;
  }
  else
  {
    settings.change_ik = true;
  }

  settings.trajectory_density = trajectory_density_->text().toInt(&conversion_ok);
  if (!conversion_ok)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid integer value in Trajectory Density field");
    settings.change_trajectory = false;
  }
  else
  {
    settings.change_trajectory = true;
  }

  settings.virtual_screw_order = screw_order_combo_->currentIndex();
  settings.cca_type = cca_type_combo_->currentIndex();

  RCLCPP_INFO(this->get_logger(), "Advanced Settings Modified");
  settings_publisher_->publish(settings);
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
  frame_place_button_->setVisible(false);
  pitch_label_->setVisible(false);
  pitch_combo_box_->setVisible(false);
  pitch_value_input_->setVisible(false);
  pitch_value_label_->setVisible(false);
  frame_place_button_->setVisible(false);
  frame_place_button_->setEnabled(false);

  // Set necessary widgets to visible
  motion_type_label_->setVisible(true);
  motion_type_combo_box_->setEnabled(true);
  motion_type_combo_box_->setVisible(true);
  motion_type_combo_box_->setCurrentText("");
  conf_place_button_->setEnabled(false);
  conf_place_button_->setVisible(true);
}

void CcaInteractiveGoals::createArrowInteractiveMarker()
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "arm0_base_link";
  int_marker.name = "arrow_marker";
  int_marker.description = "";
  int_marker.scale = 1.0;

  // Create arrow marker
  visualization_msgs::msg::Marker arrow;
  arrow.ns = "interactive_goals";
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.scale.x = 1.0;
  arrow.scale.y = 0.1;
  arrow.scale.z = 0.1;
  arrow.color.r = 0.5;
  arrow.color.g = 0.5;
  arrow.color.b = 0.5;
  arrow.color.a = 1.0;
  arrow.id = 8;

  // Create a control for the arrow
  visualization_msgs::msg::InteractiveMarkerControl arrow_control;
  arrow_control.always_visible = true;
  arrow_control.markers.push_back(arrow);
  int_marker.controls.push_back(arrow_control);

  // Create controls for movement and rotation
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker, std::bind(&CcaInteractiveGoals::processArrowFeedback, this, std::placeholders::_1));
}

void CcaInteractiveGoals::processArrowFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  switch (feedback->event_type)
  {
    case visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE:
      RCLCPP_INFO(this->get_logger(),
                  "Arrow marker moved to position (%.2f, %.2f, %.2f) and orientation(%.2f, %.2f, %.2f, %.2f)",
                  feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z,
                  feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
                  feedback->pose.orientation.w);
      server_->setPose(feedback->marker_name, feedback->pose);
      server_->applyChanges();
      break;
  }
}

void CcaInteractiveGoals::createInvisibleInteractiveMarker()
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "arm0_base_link";
  int_marker.name = "approach_frame";
  int_marker.description = "";
  int_marker.scale = 1.0;

  visualization_msgs::msg::InteractiveMarkerControl frame_control;
  frame_control.always_visible = true;

  // X axis (red)
  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CYLINDER;

  marker.scale.x = 0.05;
  marker.scale.y = 0.05;
  marker.scale.z = 0.5;
  marker.color.a = 1.0;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.pose.position.x = 0.25;
  marker.pose.orientation.w = 0.7071;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0.7071;
  marker.pose.orientation.z = 0;
  frame_control.markers.clear();
  frame_control.markers.push_back(marker);
  frame_control.orientation.w = 1;
  frame_control.orientation.x = 0;
  frame_control.orientation.y = 0;
  frame_control.orientation.z = 0;
  int_marker.controls.push_back(frame_control);

  // Y axis (green)
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0.25;
  marker.pose.orientation.w = 0.7071;
  marker.pose.orientation.x = -0.7071;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  frame_control.markers.clear();
  frame_control.markers.push_back(marker);
  frame_control.orientation.w = 0.7071;
  frame_control.orientation.x = 0;
  frame_control.orientation.y = 0;
  frame_control.orientation.z = 0.7071;
  frame_control.name = "y_axis";
  int_marker.controls.push_back(frame_control);

  // Z axis (blue)
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0.25;
  marker.pose.orientation.w = 1;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  frame_control.markers.clear();
  frame_control.markers.push_back(marker);
  frame_control.orientation.w = 0.7071;
  frame_control.orientation.x = -0.7071;
  frame_control.orientation.y = 0;
  frame_control.orientation.z = 0;
  frame_control.name = "z_axis";
  int_marker.controls.push_back(frame_control);

  // Create controls for movement and rotation
  visualization_msgs::msg::InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server_->insert(int_marker,
                  std::bind(&CcaInteractiveGoals::processInvisibleMarkerFeedback, this, std::placeholders::_1));
}

void CcaInteractiveGoals::processInvisibleMarkerFeedback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
  {
    RCLCPP_INFO(this->get_logger(),
                "Invisible marker moved to position (%.2f, %.2f, %.2f) and orientation(% .2f, % .2f, % .2f, % .2f) ",
                feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z,
                feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z,
                feedback->pose.orientation.w);

    // Update affordance axis and location
    Eigen::Quaterniond q(feedback->pose.orientation.w,
			feedback->pose.orientation.x,
			feedback->pose.orientation.y,
			feedback->pose.orientation.z);
    Eigen::Vector3d X_AXIS(1.0, 0.0, 0.0);//Starts as oriented along x
    affordance_axis_ = q * X_AXIS;
    affordance_location_ = Eigen::Vector3d(
          feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);

    // Update the marker's pose
    server_->setPose(feedback->marker_name, feedback->pose);
    server_->applyChanges();

    // Publish TF frame
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "arm0_base_link";
    transform.child_frame_id = "approach_frame"; // NOTE: This may be wrong
    transform.transform.translation.x = feedback->pose.position.x;
    transform.transform.translation.y = feedback->pose.position.y;
    transform.transform.translation.z = feedback->pose.position.z;
    transform.transform.rotation = feedback->pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }
}

void CcaInteractiveGoals::enableInteractiveMarkerControls(const std::string& marker_name)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  if (server_->get(marker_name, int_marker))
  {
    for (auto& control : int_marker.controls)
    {
      if (control.name.find("rotate") != std::string::npos)
      {
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::ROTATE_AXIS;
      }
      else if (control.name.find("move") != std::string::npos)
      {
        control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
      }

      // Make the arrow visible
      if (!control.markers.empty())
      {
        control.markers[0].color.a = 1.0;  // Set alpha to 1.0 for full opacity
      }
    }
    server_->insert(int_marker);
    server_->applyChanges();
  }
}

void CcaInteractiveGoals::disableInteractiveMarkerControls(const std::string& marker_name)
{
  visualization_msgs::msg::InteractiveMarker int_marker;
  if (server_->get(marker_name, int_marker))
  {
    for (auto& control : int_marker.controls)
    {
      control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::NONE;

      // Make the arrow invisible
      if (!control.markers.empty())
      {
        control.markers[0].color.a = 0.0;  // Set alpha to 0.0 for full transparency
      }
    }
    server_->insert(int_marker);
    server_->applyChanges();
  }
}

void CcaInteractiveGoals::spin()
{
  rclcpp::spin_some(this->get_node_base_interface());
}

std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
interactive_markers::MenuHandler menu_handler_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

}  // namespace cca_interactive_goals

void CcaInteractiveGoals::send_cca_action_goal_(){
    // Create goal	
    auto goal_ptr = std::make_shared<cca_ros_action::action::CcaRosAction>();
    goal_ptr->req = req_;

    if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "%s action server not available after waiting", cca_as_name_.c_str());
    rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal to %s action server."cca_as_name_.c_str());

    auto send_goal_options = rclcpp_action::Client<CcaRosAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&CcaInteractiveGoals::cca_action_client_goal_response_cb_, this, _1);
    send_goal_options.result_callback =
      std::bind(&CcaInteractiveGoals::cca_action_client_result_cb_, this, _1);
    this->cca_action_client_->async_send_goal(goal_ptr, send_goal_options);
}
void CcaInteractiveGoals::cca_action_client_goal_response_cb_(std::shared_future<GoalHandleCcaRosAction::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server, %s", cca_as_name_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by %s server, waiting for result", cca_as_name_.c_str());
    }
  }

  void CcaInteractiveGoals::cca_action_client_result_cb_(const GoalHandleCcaRosAction::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted by %s server", cca_as_name_.c_str());
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled by %s server", cca_as_name_.c_str());
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code from %s server", cca_as_name_.c_str());
        return;
    }
  }
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cca_interactive_goals::CcaInteractiveGoals, rviz_common::Panel)
