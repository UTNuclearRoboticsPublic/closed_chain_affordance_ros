#ifndef RVIZ_UI_PANEL_PANEL_HPP_
#define RVIZ_UI_PANEL_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

class QLabel;
class QComboBox;
class QPushButton;
class QLineEdit;

namespace cca_interactive_goals
{

class Panel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit Panel(QWidget* parent = nullptr);

  virtual void onInitialize() override;

  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void framePlaceButtonClicked();
  void planVizClicked();
  void planVizExeClicked();
  void planExeClicked();
  void stopClicked();
  void confirmPlaceClicked();
  void modeSelected(int index);
  void motionTypeSelected(int index);
  void goalSelected(int index);
  void pitchSelected(int index);
  void axisOptionSelected(int index);

private:
  void updateUIState();

  QComboBox* mode_combo_box_;
  QComboBox* motion_type_combo_box_;
  QComboBox* goal_combo_box_;
  QComboBox* pitch_combo_box_;
  QComboBox* axis_combo_box_;
  QLabel* value_label_;
  QLabel* pitch_value_label_;
  QLabel* goal_label_;
  QLabel* pitch_label_;
  QLabel* axis_label_;
  QLabel* motion_type_label_;
  QLineEdit* value_input_;
  QLineEdit* pitch_value_input_;
  QPushButton* plan_viz_button_;
  QPushButton* frame_place_button_;
  QPushButton* plan_viz_exe_button_;
  QPushButton* plan_exe_button_;
  QPushButton* stop_button_;
  QPushButton* conf_place_button_;
};

}  // namespace cca_interactive_goals

#endif  // RVIZ_UI_PANEL_PANEL_HPP_
