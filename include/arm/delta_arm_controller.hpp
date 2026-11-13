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

namespace DeltaArmRos
{

/**
 * @brief Task-space interface to the 3-DOF delta arm.
 *
 *  * set_pos : "arm/set_pos" action. Goal is a Twist (linear.x/y/z).
 *              Runs two-stage IK -> publishes motor targets for the driver.
 *  * pos     : "arm/pos" topic. Current pose, commanded target and the
 *              current/target motor angles, published continuously.
 *  * joints  : optional "arm/joints" debug topic (sensor_msgs/JointState)
 *              with the three motor angles in degrees. Only published when
 *              the 2-D visualiser requests it (publish_joints=true).
 */
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

  // rclcpp_action callbacks
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
  std::mutex goal_mutex_;

  DeltaArm::Arm arm_;
  std::mutex arm_mutex_;
  std::atomic<bool> feedback_seen_{false};

  double feedback_rate_;
  std::string feedback_frame_;
  double reach_tolerance_;
  double reach_timeout_;
  bool simulate_arrival_;
  bool enable_motion_;
  bool publish_joints_;
};

}  // namespace DeltaArmRos

#endif  // arm__DELTA_ARM_CONTROLLER_HPP_
