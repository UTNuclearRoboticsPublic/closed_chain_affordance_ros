#ifndef RVIZ_UI_PANEL_PANEL_HPP_
#define RVIZ_UI_PANEL_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

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
  void executeButtonClicked();
  void modeSelected(int index);
  void motionTypeSelected(int index);
  void fillerOptionSelected(int index);

private:
  void updateUIState();

  QComboBox* mode_combo_box_;
  QComboBox* motion_type_combo_box_;
  QComboBox* filler_combo_box_;
  QLineEdit* value_input_;
  QPushButton* execute_button_;
};

}  // namespace cca_interactive_goals

#endif  // RVIZ_UI_PANEL_PANEL_HPP_
