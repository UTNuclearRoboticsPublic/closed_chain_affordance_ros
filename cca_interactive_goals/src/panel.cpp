#include "cca_interactive_goals/panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>
#include <QString>
#include <vector>
#include <string>

namespace cca_interactive_goals
{

Panel::Panel(QWidget* parent) : rviz_common::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;

  // Mode selection dropdown

  QHBoxLayout* mode_layout = new QHBoxLayout;
  QLabel* mode_label = new QLabel("Select Mode:");
  mode_combo_box_ = new QComboBox;
  mode_combo_box_->addItems({ "Affordance Planning", "Cartesian Goal Planning",
                              "In-Place End Effector Orientation Control", "Approach Motion Task" });
  mode_layout->addWidget(mode_label);
  mode_layout->addWidget(mode_combo_box_);
  layout->addLayout(mode_layout);

  // Motion type dropdown

  QHBoxLayout* motion_type_layout = new QHBoxLayout;
  motion_type_label_ = new QLabel("Motion Type:");
  motion_type_combo_box_ = new QComboBox;
  motion_type_combo_box_->addItems({ "", "Translation", "Rotation", "Screw Motion" });
  motion_type_layout->addWidget(motion_type_label_);
  motion_type_layout->addWidget(motion_type_combo_box_);
  layout->addLayout(motion_type_layout);

  // Axis Selection Dropdown

  QHBoxLayout* axis_layout = new QHBoxLayout;
  axis_label_ = new QLabel("Axis:");
  axis_combo_box_ = new QComboBox;
  axis_combo_box_->addItems({ "", "Manual Input", "x", "y", "z", "-x", "-y", "-z" });
  axis_layout->addWidget(axis_label_);
  axis_layout->addWidget(axis_combo_box_);
  layout->addLayout(axis_layout);

  // Screw placement button

  conf_place_button_ = new QPushButton("Confirm Screw Placement");
  layout->addWidget(conf_place_button_);
  conf_place_button_->setEnabled(false);

  // Frame placement button

  frame_place_button_ = new QPushButton("Confirm Frame Placement");
  layout->addWidget(frame_place_button_);
  frame_place_button_->setEnabled(false);

  // Pitch Selection Dropdown

  QHBoxLayout* pitch_layout = new QHBoxLayout;
  pitch_label_ = new QLabel("Pitch(meters/radian):");
  pitch_combo_box_ = new QComboBox;
  pitch_combo_box_->addItems({ "", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5" });
  pitch_layout->addWidget(pitch_label_);
  pitch_layout->addWidget(pitch_combo_box_);
  layout->addLayout(pitch_layout);

  // Pitch input

  QHBoxLayout* pitch_value_layout = new QHBoxLayout;
  pitch_value_label_ = new QLabel("Pitch Value(meters/radian):");
  pitch_value_label_->setObjectName("Pitch Value Label");
  pitch_value_input_ = new QLineEdit;
  pitch_value_layout->addWidget(pitch_value_label_);
  pitch_value_layout->addWidget(pitch_value_input_);
  layout->addLayout(pitch_value_layout);

  // Goal Selection Dropdown

  QHBoxLayout* goal_layout = new QHBoxLayout;
  goal_label_ = new QLabel("Goal:");
  goal_combo_box_ = new QComboBox;
  goal_combo_box_->addItems(
      { "", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5", "0.6", "0.7", "0.8", "0.9", "1.0" });
  goal_layout->addWidget(goal_label_);
  goal_layout->addWidget(goal_combo_box_);
  layout->addLayout(goal_layout);

  // Goal input

  QHBoxLayout* value_layout = new QHBoxLayout;
  value_label_ = new QLabel("Value:");
  value_label_->setObjectName("Value Label");
  value_input_ = new QLineEdit;
  value_layout->addWidget(value_label_);
  value_layout->addWidget(value_input_);
  layout->addLayout(value_layout);

  // Plan, Viz, Execute Buttons

  plan_viz_button_ = new QPushButton("Plan + Visualize");
  layout->addWidget(plan_viz_button_);
  plan_viz_button_->setEnabled(false);
  plan_viz_exe_button_ = new QPushButton("Plan, Visualize + Execute");
  layout->addWidget(plan_viz_exe_button_);
  plan_viz_exe_button_->setEnabled(false);
  plan_exe_button_ = new QPushButton("Plan + Execute");
  layout->addWidget(plan_exe_button_);
  plan_exe_button_->setEnabled(false);
  stop_button_ = new QPushButton("STOP");
  layout->addWidget(stop_button_);
  stop_button_->setEnabled(false);

  setLayout(layout);

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

  updateUIState();
}

void Panel::onInitialize()
{
}

void Panel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
}

void Panel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void Panel::planVizClicked()
{
  // Handle plan viz button click
}

void Panel::planVizExeClicked()
{
  // Handle plan viz execute button click
}

void Panel::planExeClicked()
{
  // Handle plan execute button click
}

void Panel::stopClicked()
{
  // Handle stop button click
}

void Panel::confirmPlaceClicked()
{
  value_input_->setVisible(false);
  value_input_->setEnabled(false);
  value_label_->setVisible(false);
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
    std::vector<std::string> pi_fractions = { "π/4", "π/2", "3π/4", "π", "5π/4", "3π/2", "7π/4", "2π", "9π/4", "5π/2" };
    for (int i = 2; i <= 11; i++)
    {
      goal_combo_box_->setItemText(i, pi_fractions[i - 2].c_str());
    }
  }
}

void Panel::framePlaceButtonClicked()
{
  // Handle confirm frame place button click
}

void Panel::modeSelected(int index)
{
  // Grey out execute buttons

  plan_viz_button_->setEnabled(false);
  // execute_button_->setEnabled(false);
  plan_viz_exe_button_->setEnabled(false);
  plan_exe_button_->setEnabled(false);
  stop_button_->setEnabled(false);

  // Setup other widgets based on selection

  bool mode_selected = mode_combo_box_->currentIndex() != -1;

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

      // Set necessary widgets to visible
      motion_type_label_->setVisible(true);
      motion_type_combo_box_->setEnabled(true);
      motion_type_combo_box_->setVisible(true);
      motion_type_combo_box_->setCurrentText("");
      conf_place_button_->setEnabled(false);
      conf_place_button_->setVisible(true);

      // TODO: Spawn in arrow
    }
    else if (mode_combo_box_->currentText() == "Cartesian Goal Planning")
    {
      // TODO: set all others to not visible

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
    }
    else if (mode_combo_box_->currentText() == "In-Place End Effector Orientation Control")
    {
      // TODO: set all others to not visible

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

      axis_combo_box_->setEnabled(true);
      axis_combo_box_->setVisible(true);
      axis_label_->setVisible(true);
      axis_combo_box_->setCurrentIndex(0);
    }
  }
}

void Panel::motionTypeSelected(int index)
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
    conf_place_button_->setEnabled(true);
    conf_place_button_->setVisible(true);
  }
  else
  {
    conf_place_button_->setEnabled(false);
    conf_place_button_->setVisible(true);
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

void Panel::goalSelected(int index)
{
  if (index == 1)
  {
    value_label_->setVisible(true);
    value_input_->setEnabled(true);
    value_input_->setVisible(true);
    if (motion_type_combo_box_->currentText() == "Translation")
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

void Panel::pitchSelected(int index)
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

void Panel::axisOptionSelected(int index)
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
  }
}

void Panel::updateUIState()
{
  bool mode_selected = mode_combo_box_->currentIndex() != -1;
  motion_type_combo_box_->setEnabled(mode_selected && (mode_combo_box_->currentText() == "Affordance Planning" ||
                                                       mode_combo_box_->currentText() == "Approach Motion Task"));
  axis_combo_box_->setEnabled(mode_selected &&
                              mode_combo_box_->currentText() == "In-Place End Effector Orientation Control");

  bool show_value_input = false;
  QString value_label_ = "";

  if (mode_selected)
  {
    if (mode_combo_box_->currentText() == "Affordance Planning" ||
        mode_combo_box_->currentText() == "Approach Motion Task")
    {
      if (motion_type_combo_box_->currentText() == "Translation")
      {
        show_value_input = true;
        value_label_ = "Distance (meters):";
      }
      else if (motion_type_combo_box_->currentText() == "Rotation" ||
               motion_type_combo_box_->currentText() == "Screw Motion")
      {
        show_value_input = true;
        value_label_ = "Angle (radians):";
      }
    }
    else if (mode_combo_box_->currentText() == "In-Place End Effector Orientation Control")
    {
      if (axis_combo_box_->currentText() != "Interactive Selection")
      {
        show_value_input = true;
        value_label_ = "Angle (radians):";
      }
    }
  }

  value_input_->setEnabled(show_value_input);
  value_input_->setVisible(show_value_input);
  value_input_->parentWidget()->findChild<QLabel*>("Value Label")->setText(value_label_);

  // execute_button_->setEnabled(mode_selected);
}

}  // namespace cca_interactive_goals

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cca_interactive_goals::Panel, rviz_common::Panel)
