#ifndef arm__DELTA_ARM_CONTROLLER_HPP_
#define arm__DELTA_ARM_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm/action/set_position.hpp"
#include "arm/msg/arm_position.hpp"
#include "arm/msg/motor_feedback.hpp"
#include "arm/msg/motor_targets.hpp"
#include "arm/delta_arm.hpp"
#include "arm/trajectory_modifier.hpp"

namespace DeltaArmRos
{

/// Task-space controller: set_pos action -> IK -> motor targets and arm/pos.
class DeltaArmControllerNode : public rclcpp::Node
{
public:
  using SetPosition = arm::action::SetPosition;
  using GoalHandle = rclcpp_action::ServerGoalHandle<SetPosition>;

  explicit DeltaArmControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DeltaArmControllerNode() override;

private:
  void declareParams();
  void setupActionServer();
  void setupMotorFeedback();
  void setupPositionFeedback();
  void clearGoal(const std::shared_ptr<GoalHandle> & goal_handle);
  void publishPosition();
  void publishTargets(const float * angles);
  void onMotorFeedback(const arm::msg::MotorFeedback::SharedPtr msg);


  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SetPosition::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
  void executeAction(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_action::Server<SetPosition>::SharedPtr action_server_;
  rclcpp::CallbackGroup::SharedPtr action_cb_group_;
  rclcpp::Publisher<arm::msg::MotorTargets>::SharedPtr motor_pub_;
  rclcpp::Publisher<arm::msg::ArmPosition>::SharedPtr pos_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Subscription<arm::msg::MotorFeedback>::SharedPtr motor_fb_sub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  std::shared_ptr<GoalHandle> current_goal_handle_;
  rclcpp_action::GoalUUID active_goal_uuid_{};
  std::mutex goal_mutex_;

  DeltaArm::Arm arm_;
  std::mutex arm_mutex_;
  std::atomic<bool> feedback_seen_{false};

  DeltaArm::EndpointTrajectoryModifier trajectory_;
  DeltaArm::MotorAccelLimiter motor_limiter_;

  double feedback_rate_;
  std::string feedback_frame_;
  double reach_tolerance_;
  double reach_timeout_;
  bool simulate_arrival_;
  bool enable_motion_;
  bool publish_joints_;

  bool trajectory_enabled_;
  double trajectory_rate_;
  double endpoint_max_speed_;
  double endpoint_max_accel_;
  double motor_max_speed_;
  double motor_max_accel_;
};

}

#endif
