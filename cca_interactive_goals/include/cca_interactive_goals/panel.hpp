#ifndef RVIZ_UI_PANEL_PANEL_HPP_
#define RVIZ_UI_PANEL_PANEL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <memory>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "interactive_goal_interfaces/msg/button_press.hpp"
#include "interactive_goal_interfaces/msg/screw_info.hpp"
#include "interactive_goal_interfaces/msg/advanced_settings.hpp"
#include <Eigen/Dense>
#include <cc_affordance_planner/cc_affordance_planner.hpp>
#include <cca_ros/cca_ros.hpp>
#include <cca_ros_util/cca_ros_util.hpp>
#include <cca_ros_action/cca_ros_action.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <cca_ros_viz_msgs/action/cca_ros_action.hpp>

class QLabel;
class QComboBox;
class QPushButton;
class QLineEdit;
class QTimer;

namespace cca_interactive_goals
{

class CcaInteractiveGoals : public rviz_common::Panel, public rclcpp::Node
{
  Q_OBJECT
public:
  using CcaRosAction = cca_ros_viz_msgs::action::CcaRosAction;
  using GoalHandleCcaRosAction = rclcpp_action::ServerGoalHandle<CcaRosAction>;
  explicit CcaInteractiveGoals(QWidget* parent = nullptr);

  virtual void onInitialize() override;

  virtual void load(const rviz_common::Config& config) override;
  virtual void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void framePlaceButtonClicked();
  void planVizClicked();
  void planVizExeClicked();
  void planExeClicked();
  void stopClicked();
  void screwInfoBuilder();
  void confirmPlaceClicked();
  void modeSelected(int index);
  void motionTypeSelected(int index);
  void goalSelected(int index);
  void pitchSelected(int index);
  void axisOptionSelected(int index);
  void intMarkerController(visualization_msgs::msg::InteractiveMarker int_marker, bool rotate, bool translate);
  void createArrowInteractiveMarker();
  void processArrowFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void createInvisibleInteractiveMarker();
  void
  processInvisibleMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback);
  void enableInteractiveMarkerControls(const std::string& marker_name);
  void disableInteractiveMarkerControls(const std::string& marker_name);
  void applySettingsClicked();
  void spin();

private:
  void updateUIState();

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<interactive_goal_interfaces::msg::ScrewInfo>::SharedPtr screw_info_publisher_;
  rclcpp::Publisher<interactive_goal_interfaces::msg::ButtonPress>::SharedPtr button_press_publisher_;
  rclcpp::Publisher<interactive_goal_interfaces::msg::AdvancedSettings>::SharedPtr settings_publisher_;

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
  QTimer* spin_timer_;

  // Advanced Settings Widgets
  QLineEdit* accuracy_;
  QLineEdit* closure_angle_;
  QLineEdit* closure_linear_;
  QLineEdit* ik_iterations_;
  QLineEdit* trajectory_density_;
  QComboBox* screw_order_combo_;
  QComboBox* cca_type_combo_;
  QPushButton* apply_button_;

  // CCA parameters
  Eigen::Vector3d affordance_axis_;
  Eigen::Vector3d affordance_location_;
  cca_ros::PlanningRequest req_;

  // ROS clients
    rclcpp_action::Client<cca_ros_viz_msgs::action::CcaRosAction>::SharedPtr
        cca_action_client_; ///< Client for planning and executing with CCA
    const std::string cca_as_name_= "/cca_ros_action";

void cca_action_client_goal_response_cb_(std::shared_future<GoalHandleCcaRosAction::SharedPtr> future);
  void cca_action_client_feedback_cb_(
    GoalHandleCcaRosAction::SharedPtr,
    const std::shared_ptr<const CcaRosAction::Feedback> feedback);
  void cca_action_client_result_cb_(const GoalHandleCcaRosAction::WrappedResult & result);
};

}  // namespace cca_interactive_goals

#endif  // RVIZ_UI_PANEL_PANEL_HPP_
