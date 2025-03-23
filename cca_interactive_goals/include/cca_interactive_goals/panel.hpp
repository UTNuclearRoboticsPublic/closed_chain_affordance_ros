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
    enum CcaType
    {
        AFFORDANCE_CONTROL,
        AFFORDANCE_AND_EE_ORIENTATION_CONTROL
    };
    struct QComboBoxAndLabel
    {
        QComboBox *combo_box;
        QLabel *label;
    };

    struct QLineEditAndLabel
    {
        QLineEdit *line_edit;
        QLabel *label;
    };
    struct AdvancedSettingsWidgets
    {
        QLineEdit *accuracy;
        QLineEdit *closure_angle;
        QLineEdit *closure_linear;
        QLineEdit *ik_iterations;
        QLineEdit *trajectory_density;
        QComboBox *cca_type;
        QComboBox *vir_screw_order;
    };
    struct AdvancedSettings
    {
        struct TaskDescription
        {
            int trajectory_density = 10;
            std::optional<affordance_util::VirtualScrewOrder> vir_screw_order;
        };
        cc_affordance_planner::PlannerConfig planner_config;
        TaskDescription task_description;
        CcaType cca_type;
    };

    explicit CcaInteractiveGoals(QWidget *parent = nullptr);

    virtual void onInitialize() override;

    virtual void load(const rviz_common::Config &config) override;
    virtual void save(rviz_common::Config config) const override;

  protected Q_SLOTS:
    void plan_button_clicked_();
    void plan_exe_button_clicked_();
    void exe_button_clicked_();
    void cancel_exe_button_clicked_();
    cca_ros::PlanningRequest buildPlanningRequest();
    void mode_selected_();
    void motion_type_selected_();
    void goal_selected_(int index);
    void pitch_selected_(int index);
    void axis_option_selected_(QString axis);
    void apply_settings_clicked_();
    void spin();

  private:
    const QStringList AXES_ = {"", "Interactive Axis", "x", "y", "z", "-x", "-y", "-z"};
    const QStringList PITCHES_ = {"", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5"};
    const QStringList TRANSLATION_GOALS_ = {"",    "Manual Input", "0.1", "0.2", "0.3", "0.4",
                                            "0.5", "0.6",          "0.7", "0.8", "0.9", "1.0"};
    const QStringList ROTATION_GOALS_ = {"", "Manual Input", "π/4", "π/2", "3π/4", "π", "5π/4", "3π/2", "7π/4", "2π"};

    const std::map<QString, affordance_util::VirtualScrewOrder> vir_screw_order_map_ = {
        {QString("NONE"), affordance_util::VirtualScrewOrder::NONE},
        {QString("YZX"), affordance_util::VirtualScrewOrder::YZX},
        {QString("ZXY"), affordance_util::VirtualScrewOrder::ZXY},
        {QString("XY"), affordance_util::VirtualScrewOrder::XY},
        {QString("YZ"), affordance_util::VirtualScrewOrder::YZ},
        {QString("ZX"), affordance_util::VirtualScrewOrder::ZX},
        {QString("XYZ"), affordance_util::VirtualScrewOrder::XYZ}}; // virtual screw order to Qstring map for the
                                                                    // advanced settings menu
    const std::map<QString, CcaType> cca_type_map_ = {
        {QString("Affordance Control"), CcaType::AFFORDANCE_CONTROL},
        {QString("Affordance and EE Orientation Control"), CcaType::AFFORDANCE_AND_EE_ORIENTATION_CONTROL}};

    const std::map<QString, cc_affordance_planner::PlanningType> planning_type_map_ = {
        {QString("Affordance"), cc_affordance_planner::PlanningType::AFFORDANCE},
        {QString("EE Orientation Only"), cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY}};
    // {QString("Cartesian Goal"), cc_affordance_planner::PlanningType::CARTESIAN_GOAL}, /// FUTURE WORK
    // {QString("Approach"), cc_affordance_planner::PlanningType::APPROACH}}; /// FUTURE WORK

    const std::map<QString, affordance_util::ScrewType> motion_type_map_ = {
        {QString("Translation"), affordance_util::ScrewType::TRANSLATION},
        {QString("Rotation"), affordance_util::ScrewType::ROTATION},
        {QString("Screw"), affordance_util::ScrewType::SCREW}};

    static constexpr int PANEL_WIDTH_ = 400;
    static constexpr int PANEL_HEIGHT_ = 475;
    bool new_settings_applied_ = false;

    QComboBoxAndLabel mode_bl_;
    QComboBoxAndLabel motion_type_bl_;
    QComboBoxAndLabel axis_bl_;
    QComboBoxAndLabel pitch_bl_;
    QComboBoxAndLabel goal_bl_;
    QLineEditAndLabel value_ll_;
    QLineEditAndLabel pitch_value_ll_;
    QPushButton *plan_viz_button_;
    QPushButton *plan_viz_exe_button_;
    QPushButton *plan_exe_button_;
    QPushButton *stop_button_;
    QTimer *spin_timer_;

    // Advanced Settings Widgets
    AdvancedSettingsWidgets advanced_settings_widgets_;
    QPushButton *apply_button_;

    // CCA-related variables
    AdvancedSettings advanced_settings_;
    std::shared_ptr<cca_ros_action::CcaRosActionClient> ccaRosActionClient;

    // Methods
    double getAffordanceGoal_();
    void update_ui_state_();
    QWidget *create_cca_ig_tab_();
    QWidget *create_advanced_settings_tab_();
    void create_button_(QPushButton *&button, const QString &text, QVBoxLayout *layout);
    QHBoxLayout *create_combo_box_layout_(const QString &label, QComboBox *&combo_box, const QStringList &items);
    QHBoxLayout *create_combo_box_layout_(const QString &label, QComboBoxAndLabel &bl, const QStringList &items);
    QHBoxLayout *create_line_edit_layout_(const QString &label, QLineEditAndLabel &ll);
    void connect_signals_();
    template <typename ValueType>
    QStringList get_map_keys_(const std::map<QString, ValueType> &map, bool prepend_empty = false);
};

} // namespace cca_interactive_goals

#endif // RVIZ_UI_PANEL_PANEL_HPP_
