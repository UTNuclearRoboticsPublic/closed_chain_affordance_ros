#ifndef CCA_ROS_RVIZ_PLUGIN_HPP_
#define CCA_ROS_RVIZ_PLUGIN_HPP_

// Standard C++ headers
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <optional>
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
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/color_rgba.hpp>

// Qt headers
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

// Local headers
#include "cca_interactive_goals/interactive_marker_manager.hpp"

namespace cca_ros_rviz_plugin
{

/**
 * @class CcaRosRvizPlugin
 * @brief Interactive UI panel for RViz to control and visualize CCA planning.
 *
 * This panel provides a user interface for configuring and executing constrained motion
 * planning tasks using the Closed-Chain Affordance (CCA) planner. It allows users
 * to set up screw-based affordance planning and in-place EE orientation control.
 */
class CcaRosRvizPlugin : public rviz_common::Panel, public interactive_marker_manager::InteractiveMarkerManager
{
    Q_OBJECT
    // The Q_OBJECT macro marks this class as a Qt object. It enables Qt's signal-slot mechanism,
    // property system, and other meta-object features needed for QObject-derived classes.
  public:
    /**
     * @brief Enum defining the CCA control type
     */
    enum CcaType
    {
        AFFORDANCE_CONTROL,                   ///< Control affordances while allowing minute deviation in EE orientation
        AFFORDANCE_AND_EE_ORIENTATION_CONTROL ///< Control both affordance as well as EE orientation
    };

    /**
     * @brief Struct to group QComboBox and its label
     */
    struct QComboBoxAndLabel
    {
        QComboBox *combo_box; ///< The combo box widget
        QLabel *label;        ///< The label widget
    };

    /**
     * @brief Struct to group QLineEdit and its label
     */
    struct QLineEditAndLabel
    {
        QLineEdit *line_edit; ///< The line edit widget
        QLabel *label;        ///< The label widget
    };

    /**
     * @brief Struct to store widgets for advanced settings
     */
    struct AdvancedSettingsWidgets
    {
        QLineEdit *accuracy;                  ///< Planner accuracy
        QLineEdit *closure_angular_threshold; ///< Closed-Chain closure angular error threshold
        QLineEdit *closure_linear_threshold;  ///< Closed-Chain closure angular error threshold
        QLineEdit *ik_iterations;             ///< Maximum IK iterations
        QLineEdit *trajectory_density;        ///< No. of points in the trajectory
        QComboBox *cca_type;                  ///< Selector for CCA type
        QComboBox *vir_screw_order;           ///< Selector for virtual screw order
    };

    /**
     * @brief Struct to store advanced planning settings
     */
    struct AdvancedSettings
    {
        /**
         * @brief Struct for task-specific advanced settings
         */
        struct TaskDescription
        {
            int trajectory_density = 10;                                       ///< trajectory density
            std::optional<affordance_util::VirtualScrewOrder> vir_screw_order; ///< virtual screw order
        };

        cc_affordance_planner::PlannerConfig planner_config; ///< Configuration for the CCA planner
        TaskDescription task_description;                    ///< Task description advanced settings
        CcaType cca_type;                                    ///< CCA type
    };

    /**
     * @brief Constructor for the CcaRosRvizPlugin panel
     * @param parent Parent widget
     */
    explicit CcaRosRvizPlugin(QWidget *parent = nullptr);

    /**
     * @brief Initialize ROS connections and set up the panel
     *
     * This method is called after the panel is created to initialize ROS connections,
     * action clients, and timers for node spinning.
     */
    virtual void onInitialize() override;

    /**
     * @brief Load panel configuration
     * @param config Panel configuration
     */
    inline void load(const rviz_common::Config &config) override { rviz_common::Panel::load(config); }

    /**
     * @brief Save panel configuration
     * @param config Panel configuration to save to
     */
    inline void save(rviz_common::Config config) const override { rviz_common::Panel::save(config); }

    /**
     * @brief ROS node spinning function
     *
     * Processes ROS callbacks and messages.
     */
    inline void spin() { rclcpp::spin_some(this->get_node_base_interface()); }

  private Q_SLOTS:
    // The Q_SLOTS macro marks methods as slots. These are basically callbacks to various UI buttons or signals.
    /**
     * @brief Handle plan visualization button click
     *
     * Plans a trajectory and visualizes it without execution.
     */
    void plan_button_clicked_();

    /**
     * @brief Handle plan and execute button click
     *
     * Plans a trajectory, visualizes it, and executes it.
     */
    void plan_exe_button_clicked_();

    /**
     * @brief Handle execution button click
     *
     * Executes a planned trajectory without visualization.
     */
    void exe_button_clicked_();

    /**
     * @brief Handle cancellation button click
     *
     * Cancels the current trajectory execution.
     */
    void cancel_exe_button_clicked_();

    /**
     * @brief Handle planning mode selection change
     *
     * Updates the UI when user selects a different planning mode.
     */
    void mode_selected_();

    /**
     * @brief Handle motion type selection change
     *
     * Updates the UI when user selects a different motion type.
     */
    void motion_type_selected_();

    /**
     * @brief Handle goal selection change
     * @param index The index of the selected goal
     *
     * Updates the UI when user selects a different goal value.
     */
    void goal_selected_(int index);

    /**
     * @brief Handle pitch selection change
     * @param index The index of the selected pitch
     *
     * Updates the UI when user selects a different pitch value for screw motion.
     */
    void pitch_selected_(int index);

    /**
     * @brief Handle axis selection change
     * @param axis The selected axis
     *
     * Updates the UI and interactive markers when user selects a different axis.
     */
    void axis_option_selected_(QString axis);

    /**
     * @brief Handle apply settings button click
     *
     * Extracts the advanced settings from the UI .
     */
    void apply_settings_clicked_();

  private:
    // UI constants
    static constexpr int PANEL_WIDTH_ = 400;   ///< Panel width in pixels
    static constexpr int PANEL_HEIGHT_ = 475;  ///< Panel height in pixels
    static constexpr int LAYOUT_SPACING_ = 10; ///< Layout spacing in pixels

    // UI option lists
    const QStringList axes_ = {"", "Interactive Axis", "x", "y", "z", "-x", "-y", "-z"};
    const QStringList pitches_ = {"", "Manual Input", "0.1", "0.2", "0.3", "0.4", "0.5"};
    const QStringList translation_goals_ = {"",    "Manual Input", "0.1", "0.2", "0.3", "0.4",
                                            "0.5", "0.6",          "0.7", "0.8", "0.9", "1.0"};
    const QStringList rotation_goals_ = {"", "Manual Input", "π/4", "π/2", "3π/4", "π", "5π/4", "3π/2", "7π/4", "2π"};

    // Mapping tables for converting UI selections to internal values
    const std::map<QString, affordance_util::VirtualScrewOrder> vir_screw_order_map_ = {
        {QString("NONE"), affordance_util::VirtualScrewOrder::NONE},
        {QString("YZX"), affordance_util::VirtualScrewOrder::YZX},
        {QString("ZXY"), affordance_util::VirtualScrewOrder::ZXY},
        {QString("XY"), affordance_util::VirtualScrewOrder::XY},
        {QString("YZ"), affordance_util::VirtualScrewOrder::YZ},
        {QString("ZX"), affordance_util::VirtualScrewOrder::ZX},
        {QString("XYZ"), affordance_util::VirtualScrewOrder::XYZ}};

    const std::map<QString, CcaType> cca_type_map_ = {
        {QString("Affordance Control"), CcaType::AFFORDANCE_CONTROL},
        {QString("Affordance and EE Orientation Control"), CcaType::AFFORDANCE_AND_EE_ORIENTATION_CONTROL}};

    const std::map<QString, cc_affordance_planner::PlanningType> planning_type_map_ = {
        {QString("Affordance"), cc_affordance_planner::PlanningType::AFFORDANCE},
        {QString("EE Orientation Only"), cc_affordance_planner::PlanningType::EE_ORIENTATION_ONLY}};

    const std::map<QString, affordance_util::ScrewType> motion_type_map_ = {
        {QString("Translation"), affordance_util::ScrewType::TRANSLATION},
        {QString("Rotation"), affordance_util::ScrewType::ROTATION},
        {QString("Screw"), affordance_util::ScrewType::SCREW}};

    // State flags
    bool new_settings_applied_ = false; ///< Flag to track if advanced settings have been applied

    // UI components - main controls
    QComboBoxAndLabel mode_bl_;        ///< Planning mode selection
    QComboBoxAndLabel motion_type_bl_; ///< Motion type selection
    QComboBoxAndLabel axis_bl_;        ///< Axis selection
    QComboBoxAndLabel pitch_bl_;       ///< Pitch selection for screw motion
    QComboBoxAndLabel goal_bl_;        ///< Goal value selection
    QLineEditAndLabel value_ll_;       ///< Custom value input
    QLineEditAndLabel pitch_value_ll_; ///< Custom pitch input

    // UI components - buttons
    QPushButton *plan_viz_button_;     ///< Button for plan visualization
    QPushButton *plan_viz_exe_button_; ///< Button for plan, visualize and execute
    QPushButton *plan_exe_button_;     ///< Button for execute only
    QPushButton *stop_button_;         ///< Button for stopping execution
    QPushButton *apply_button_;        ///< Button for applying advanced settings

    // Timer for ROS spinning
    QTimer *spin_timer_; ///< Timer for ROS message processing

    // Advanced settings components
    AdvancedSettingsWidgets advanced_settings_widgets_; ///< Widgets for advanced settings
    AdvancedSettings advanced_settings_;                ///< Current advanced settings

    // ROS components
    std::shared_ptr<cca_ros_action::CcaRosActionClient>
        ccaRosActionClient; ///< ROS action client for CCA planning and execution

    /**
     * @brief Planning Request Creation Methods
     */

    /**
     * @brief Builds a planning request from the current UI state
     * @return The constructed planning request
     */
    cca_ros::PlanningRequest build_planning_request_();
    /**
     * @brief Get the affordance goal value from the UI
     * @return The goal value as a double
     */

    double get_affordance_goal_();

    /**
     * @brief UI Creation Methods
     */

    /**
     * @brief Create the main CCA Rviz Plugin tab
     * @return Pointer to the created widget
     */
    QWidget *create_cca_ig_tab_();

    /**
     * @brief Create the advanced settings tab
     * @return Pointer to the created widget
     */
    QWidget *create_advanced_settings_tab_();

    /**
     * @brief Create a button with the given text and add it to the layout
     * @param button Button pointer to initialize
     * @param text Button text
     * @param layout Layout to add the button to
     */
    void create_button_(QPushButton *&button, const QString &text, QVBoxLayout *layout);

    /**
     * @brief Create a combo box with label and add it to a layout
     * @param label Label text
     * @param combo_box Combo box pointer to initialize
     * @param items List of items for the combo box
     * @return Layout containing the label and combo box
     */
    QHBoxLayout *create_combo_box_layout_(const QString &label, QComboBox *&combo_box, const QStringList &items);

    /**
     * @brief Create a combo box with label and add it to a layout
     * @param label Label text
     * @param bl QComboBoxAndLabel structure to initialize
     * @param items List of items for the combo box
     * @return Layout containing the label and combo box
     */
    QHBoxLayout *create_combo_box_layout_(const QString &label, QComboBoxAndLabel &bl, const QStringList &items);

    /**
     * @brief Create a line edit with label and add it to a layout
     * @param label Label text
     * @param ll QLineEditAndLabel structure to initialize
     * @return Layout containing the label and line edit
     */
    QHBoxLayout *create_line_edit_layout_(const QString &label, QLineEditAndLabel &ll);

    /**
     * @brief UI State Management Methods
     */

    /**
     * @brief Connect all signal-slot pairs for UI interaction
     */
    void connect_signals_();

    /**
     * @brief Get the keys from a map as a QStringList
     * @param map The map to extract keys from
     * @param prepend_empty Whether to prepend an empty string to the list
     * @return List of keys as strings
     */
    template <typename ValueType>
    QStringList get_map_keys_(const std::map<QString, ValueType> &map, bool prepend_empty = false);

    /**
     * @brief Update the UI state based on current selections
     *
     * Resets and configures the UI components based on current state.
     */
    void update_ui_state_();

    /**
     * @brief Enable or disable execution buttons
     * @param enabled Whether buttons should be enabled
     */
    void set_execute_buttons_enabled_(bool enabled);

    /**
     * @brief Hide all control widgets
     */
    void hide_all_controls_();

    /**
     * @brief Set visibility of combo box controls
     * @param controls The controls to modify
     * @param visible Whether controls should be visible
     */
    void set_combo_box_controls_(const QComboBoxAndLabel &controls, bool visible);

    /**
     * @brief Set visibility of line edit controls
     * @param controls The controls to modify
     * @param visible Whether controls should be visible
     */
    void set_line_edit_controls_(const QLineEditAndLabel &controls, bool visible);

    /**
     * @brief Configure goal controls based on motion type
     * @param motion_type The selected motion type
     */
    void setup_goal_controls_for_motion_type_(const QString &motion_type);

    /**
     * @brief Update the value label text based on current selections
     */
    void setup_value_label_text_();

    /**
     * @brief Update execution button states based on current selections
     * @param goal_index The selected goal index
     */
    void update_execution_buttons_state_(int goal_index);
};

} // namespace cca_ros_rviz_plugin

#endif // CCA_ROS_RVIZ_PLUGIN_HPP_
