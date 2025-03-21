#ifndef RVIZ_UI_PANEL_PANEL_HPP_
#define RVIZ_UI_PANEL_PANEL_HPP_

// CPP headers
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

// CCA headers
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cca_ros/cca_ros.hpp>
#include <cca_ros_action/cca_ros_action.hpp>

// ROS headers
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "cca_interactive_goals/interactive_marker_manager.hpp"

class QLabel;
class QComboBox;
class QPushButton;
class QLineEdit;
class QTimer;

namespace cca_interactive_goals
{

class CcaInteractiveGoals : public rviz_common::Panel, public interactive_marker_manager::InteractiveMarkerManager
{
    Q_OBJECT
  public:
    struct AdvancedSettings
    {
        struct TaskDescription
        {
            int trajectory_density = 10;
            std::optional<affordance_util::VirtualScrewOrder> vir_screw_order;
        };
        cc_affordance_planner::PlannerConfig planner_config;
        TaskDescription task_description;
    };

    explicit CcaInteractiveGoals(QWidget *parent = nullptr);

    virtual void onInitialize() override;

    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;

  protected Q_SLOTS:
    void planVizClicked();
    void planVizExeClicked();
    void planExeClicked();
    void stopClicked();
    cca_ros::PlanningRequest buildPlanningRequest();
    void modeSelected(int index);
    void motionTypeSelected(int index);
    void goalSelected(int index);
    void pitchSelected(int index);
    void axisOptionSelected(QString axis);
    void applySettingsClicked();
    void spin();

  private:
    const int marker_id_ = 8;
    void updateUIState();

    QComboBox *mode_combo_box_;
    QComboBox *motion_type_combo_box_;
    QComboBox *goal_combo_box_;
    QComboBox *pitch_combo_box_;
    QComboBox *axis_combo_box_;
    QLabel *value_label_;
    QLabel *pitch_value_label_;
    QLabel *goal_label_;
    QLabel *pitch_label_;
    QLabel *axis_label_;
    QLabel *motion_type_label_;
    QLineEdit *value_input_;
    QLineEdit *pitch_value_input_;
    QPushButton *plan_viz_button_;
    QPushButton *plan_viz_exe_button_;
    QPushButton *plan_exe_button_;
    QPushButton *stop_button_;
    QPushButton *conf_place_button_;
    QTimer *spin_timer_;

    // Advanced Settings Widgets
    QLineEdit *accuracy_;
    QLineEdit *closure_angle_;
    QLineEdit *closure_linear_;
    QLineEdit *ik_iterations_;
    QLineEdit *trajectory_density_;
    QComboBox *screw_order_combo_;
    QComboBox *cca_type_combo_;
    QPushButton *apply_button_;

    const std::map<QString, affordance_util::VirtualScrewOrder> virtual_screw_order_map_ = {
        {QString("NONE"), affordance_util::VirtualScrewOrder::NONE},
        {QString("YZX"), affordance_util::VirtualScrewOrder::YZX},
        {QString("ZXY"), affordance_util::VirtualScrewOrder::ZXY},
        {QString("XY"), affordance_util::VirtualScrewOrder::XY},
        {QString("YZ"), affordance_util::VirtualScrewOrder::YZ},
        {QString("ZX"), affordance_util::VirtualScrewOrder::ZX},
        {QString("XYZ"), affordance_util::VirtualScrewOrder::XYZ}}; // virtual screw order to Qstring map for the
                                                                    // advanced settings menu

    // CCA parameters
    AdvancedSettings advanced_settings_;

    double getAffordanceGoal_();
    bool new_settings_applied_ = false;
    std::shared_ptr<cca_ros_action::CcaRosActionClient> ccaRosActionClient;
};

} // namespace cca_interactive_goals

#endif // RVIZ_UI_PANEL_PANEL_HPP_
