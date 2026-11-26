#ifndef arm__DELTA_ARM_MANUAL_HPP_
#define arm__DELTA_ARM_MANUAL_HPP_

#include <array>
#include <atomic>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm/action/set_position.hpp"
#include "arm/msg/arm_position.hpp"

namespace DeltaArmRos
{

/**
 * @brief Manual (keyboard) control node.
 *
 *  * Action-client to "arm/set_pos": remote controls the arm.
 *  * Subscribes "arm/pos" and prints current-position changes.
 *
 * Every step is relative to the LAST MEASURED pose from arm/pos, not to an
 * accumulated commanded target. A target the controller refuses therefore
 * never becomes the start of the next step, so holding a key cannot walk the
 * target deeper into an invalid area. Optional pos_min/pos_max parameters add
 * a hard task-space clamp on top of that.
 *
 * The actual key decoding lives in the standalone main
 * (delta_arm_manual_main.cpp); this class exposes the ROS plumbing.
 */
class ManualControlNode : public rclcpp::Node
{
public:
  using SetPosition = arm::action::SetPosition;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SetPosition>;

  explicit ManualControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Shift the measured pose by a task-space delta (metres) and command
  ///        that absolute target, clamped to the configured position window.
  void shift_target(const std::array<double, 3> & delta);

  /// Last position estimate received from the controller.
  std::atomic<double> pos_x_;
  std::atomic<double> pos_y_;
  std::atomic<double> pos_z_;
  std::atomic<bool> position_received_;

private:
  /// @brief Fire-and-forget an absolute task-space target.
  void send_target(double x, double y, double z);
  void onPosition(const arm::msg::ArmPosition::SharedPtr msg);
  void onGoalResult(const GoalHandle::WrappedResult & result);
  void clamp_target(std::array<double, 3> & target) const;

  rclcpp_action::Client<SetPosition>::SharedPtr action_client_;
  rclcpp::Subscription<arm::msg::ArmPosition>::SharedPtr pos_sub_;

  std::atomic<double> last_x_;
  std::atomic<double> last_y_;
  std::atomic<double> last_z_;
  std::atomic<bool> printed_once_;

  double target_x_;
  double target_y_;
  double target_z_;
  bool target_initialized_;

  std::array<double, 3> pos_min_;
  std::array<double, 3> pos_max_;
  bool clamp_enabled_;
};

}  // namespace DeltaArmRos

#endif  // arm__DELTA_ARM_MANUAL_HPP_