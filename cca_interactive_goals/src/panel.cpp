#include "cca_interactive_goals/panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPushButton>
#include <QLineEdit>

namespace cca_interactive_goals
{

Panel::Panel(QWidget* parent) : rviz_common::Panel(parent)
{
  QVBoxLayout* layout = new QVBoxLayout;

  QHBoxLayout* mode_layout = new QHBoxLayout;
  QLabel* mode_label = new QLabel("Select Mode:");
  mode_combo_box_ = new QComboBox;
  mode_combo_box_->addItems({ "A", "B", "C", "D" });
  mode_layout->addWidget(mode_label);
  mode_layout->addWidget(mode_combo_box_);
  layout->addLayout(mode_layout);

  QHBoxLayout* motion_type_layout = new QHBoxLayout;
  QLabel* motion_type_label = new QLabel("Motion Type:");
  motion_type_combo_box_ = new QComboBox;
  motion_type_combo_box_->addItems({ "Translation", "Rotation", "Screw Motion" });
  motion_type_layout->addWidget(motion_type_label);
  motion_type_layout->addWidget(motion_type_combo_box_);
  layout->addLayout(motion_type_layout);

  QHBoxLayout* filler_layout = new QHBoxLayout;
  QLabel* filler_label = new QLabel("Filler:");
  filler_combo_box_ = new QComboBox;
  filler_combo_box_->addItems({ "x", "y", "z", "Interactive Selection" });
  filler_layout->addWidget(filler_label);
  filler_layout->addWidget(filler_combo_box_);
  layout->addLayout(filler_layout);

  QHBoxLayout* value_layout = new QHBoxLayout;
  QLabel* value_label = new QLabel("Value:");
  value_label->setObjectName("Value Label");
  value_input_ = new QLineEdit;
  value_layout->addWidget(value_label);
  value_layout->addWidget(value_input_);
  layout->addLayout(value_layout);

  // value_layout->

  execute_button_ = new QPushButton("Execute");
  layout->addWidget(execute_button_);

  setLayout(layout);

  connect(mode_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(modeSelected(int)));
  connect(motion_type_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(motionTypeSelected(int)));
  connect(filler_combo_box_, SIGNAL(currentIndexChanged(int)), this, SLOT(fillerOptionSelected(int)));
  connect(execute_button_, SIGNAL(clicked()), this, SLOT(executeButtonClicked()));

  updateUIState();
}

void Panel::onInitialize()
{
  // Any additional initialization can be done here
}

void Panel::load(const rviz_common::Config& config)
{
  rviz_common::Panel::load(config);
  // Load saved values if needed
}

void Panel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  // Save current values if needed
}

void Panel::executeButtonClicked()
{
  // Handle execute button click
  // Implement your execution logic here
}

void Panel::modeSelected(int index)
{
  updateUIState();
}

void Panel::motionTypeSelected(int index)
{
  updateUIState();
}

void Panel::fillerOptionSelected(int index)
{
  updateUIState();
}

void Panel::updateUIState()
{
  bool mode_selected = mode_combo_box_->currentIndex() != -1;
  motion_type_combo_box_->setEnabled(mode_selected &&
                                     (mode_combo_box_->currentText() == "A" || mode_combo_box_->currentText() == "D"));
  filler_combo_box_->setEnabled(mode_selected && mode_combo_box_->currentText() == "C");

  bool show_value_input = false;
  QString value_label = "";

  if (mode_selected)
  {
    if (mode_combo_box_->currentText() == "A" || mode_combo_box_->currentText() == "D")
    {
      if (motion_type_combo_box_->currentText() == "Translation")
      {
        show_value_input = true;
        value_label = "Distance (meters):";
      }
      else if (motion_type_combo_box_->currentText() == "Rotation" ||
               motion_type_combo_box_->currentText() == "Screw Motion")
      {
        show_value_input = true;
        value_label = "Angle (radians):";
      }
    }
    else if (mode_combo_box_->currentText() == "C")
    {
      if (filler_combo_box_->currentText() != "Interactive Selection")
      {
        show_value_input = true;
        value_label = "Angle (radians):";
      }
    }
  }

  value_input_->setEnabled(show_value_input);
  value_input_->setVisible(show_value_input);
  value_input_->parentWidget()->findChild<QLabel*>("Value Label")->setText(value_label);

  execute_button_->setEnabled(mode_selected);
}

}  // namespace cca_interactive_goals

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cca_interactive_goals::Panel, rviz_common::Panel)
