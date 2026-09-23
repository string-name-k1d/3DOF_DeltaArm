#ifndef arm__MOTOR_DRIVER_NODE_HPP_
#define arm__MOTOR_DRIVER_NODE_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "arm/msg/arm_feedback.hpp"
#include "arm/msg/motor_targets.hpp"
#include "arm/srv/motor_param_query.hpp"
#include "arm/motor_driver.hpp"

namespace DeltaArmDriverNode
{

/**
 * @brief Bridges ROS traffic to the physical FashionStart bus-servos.
 *
 *  * subscribes "arm/motor_targets"  -> applies joint angles to the motors
 *  * services  "arm/motor_param_query" -> generic hardware/param query
 *    (servo_id, request_type -> response_type, value)
 */
class MotorDriverNode : public rclcpp::Node
{
public:
  explicit MotorDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParams();
  void onMotorTargets(const arm::msg::MotorTargets::SharedPtr msg);
  void onParamQuery(
    const std::shared_ptr<arm::srv::MotorParamQuery::Request> request,
    const std::shared_ptr<arm::srv::MotorParamQuery::Response> response);
  void publishFeedback();
  void handleServoFault(int index, int error_code);

  std::shared_ptr<DeltaArmDriver::MotorDriver> driver_;
  rclcpp::Subscription<arm::msg::MotorTargets>::SharedPtr target_sub_;
  rclcpp::Service<arm::srv::MotorParamQuery>::SharedPtr query_srv_;
  rclcpp::Publisher<arm::msg::ArmFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  double feedback_rate_;
  std::string feedback_frame_;
  int8_t online_[3];
  int8_t error_[3];
  bool enable_motion_;

  /// Input target topic. Default "arm/motor_targets": the controller applies
  /// the control augmenter (offset/feedforward/clamp) to every message in
  /// publishTargets() before publishing, so the driver consumes the already
  /// augmented stream directly (see control_augmenter.hpp). The parameter
  /// exists for external controllers that publish elsewhere.
  std::string targets_topic_;

  // ── Feedback debounce ────────────────────────────────────────────────────
  // A servo is only declared offline after `offline_streak_` CONSECUTIVE
  // failed pings, and a failed angle query holds the last good value instead
  // of publishing -1/offline. Without this, a single serial-bus blip flips
  // the servo state, and any consumer that switches between the measured
  // angles and a model estimate (the 2-D visualiser) visibly shakes.
  int ping_fail_streak_[3] = {0, 0, 0};
  int query_fail_streak_[3] = {0, 0, 0};
  float last_good_angle_[3] = {-1.0f, -1.0f, -1.0f};
  int offline_streak_ = 3;

  /// Guards all access to driver_ : the node runs on a MultiThreadedExecutor,
  /// so the feedback timer and the motor-targets subscription / query service
  /// may be called concurrently and must not touch the shared serial port at
  /// the same time.
  std::mutex bus_mutex_;
};

}  // namespace DeltaArmDriverNode

#endif  // arm__MOTOR_DRIVER_NODE_HPP_