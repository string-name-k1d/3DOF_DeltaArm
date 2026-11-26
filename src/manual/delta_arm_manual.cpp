#include "arm/delta_arm_manual.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace DeltaArmRos
{

ManualControlNode::ManualControlNode(const rclcpp::NodeOptions & options)
  : Node("arm_manual_control", options),
    pos_x_(0.0), pos_y_(0.0), pos_z_(0.0),
    position_received_(false),
    last_x_(0.0), last_y_(0.0), last_z_(0.0),
    printed_once_(false),
    target_x_(0.0), target_y_(0.0), target_z_(0.0),
    target_initialized_(false),
    pos_min_{{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity()}},
    pos_max_{{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
              -std::numeric_limits<double>::infinity()}},
    clamp_enabled_(false)
{
  action_client_ = rclcpp_action::create_client<SetPosition>(this, "arm/set_pos");

  pos_sub_ = create_subscription<arm::msg::ArmPosition>(
    "arm/pos", 10,
    [this](const arm::msg::ArmPosition::SharedPtr msg) { onPosition(msg); });

  const std::vector<double> lo = declare_parameter<std::vector<double>>("pos_min", std::vector<double>{});
  const std::vector<double> hi = declare_parameter<std::vector<double>>("pos_max", std::vector<double>{});
  if (lo.size() == 3 && hi.size() == 3) {
    clamp_enabled_ = true;
    for (size_t i = 0; i < 3; ++i) {
      pos_min_[i] = lo[i];
      pos_max_[i] = hi[i];
      if (!std::isfinite(pos_min_[i]) || !std::isfinite(pos_max_[i]) || pos_min_[i] >= pos_max_[i]) {
        clamp_enabled_ = false;
      }
    }
  }

  RCLCPP_INFO(get_logger(),
              "arm_manual_control ready (waiting for set_pos server; steps start from measured pose%s)",
              clamp_enabled_ ? ", clamped to pos_min/pos_max" : "");
}

void ManualControlNode::shift_target(const std::array<double, 3> & delta)
{
  // A rejected goal must not become the origin of the next step: otherwise
  // repeated keypresses keep walking the target deeper into the invalid area
  // and the arm stays still until enough reverse steps bring it back. Start
  // every step from where the arm actually is. Before the first feedback
  // arrives, fall back to the last commanded target so keys are not lost.
  std::array<double, 3> base{{pos_x_.load(), pos_y_.load(), pos_z_.load()}};
  if (!position_received_.load() && target_initialized_) {
    base = {{target_x_, target_y_, target_z_}};
  }

  std::array<double, 3> target{{base[0] + delta[0], base[1] + delta[1], base[2] + delta[2]}};
  clamp_target(target);
  send_target(target[0], target[1], target[2]);
}

void ManualControlNode::clamp_target(std::array<double, 3> & target) const
{
  if (!clamp_enabled_) return;
  for (size_t i = 0; i < 3; ++i) {
    target[i] = std::clamp(target[i], pos_min_[i], pos_max_[i]);
  }
}

void ManualControlNode::send_target(double x, double y, double z)
{
  target_x_ = x;
  target_y_ = y;
  target_z_ = z;
  target_initialized_ = true;

  if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "set_pos action server not available yet");
    return;
  }

  auto goal = SetPosition::Goal();
  goal.target.linear.x = x;
  goal.target.linear.y = y;
  goal.target.linear.z = z;

  // Fire-and-forget: the controller accepts & executes immediately, and a newer
  // goal simply updates the target. The result callback only logs/steers the
  // operator - it never changes target_*, which is owned by the key thread.
  auto options = rclcpp_action::Client<SetPosition>::SendGoalOptions();
  options.result_callback = std::bind(&ManualControlNode::onGoalResult, this, std::placeholders::_1);
  action_client_->async_send_goal(goal, options);

  RCLCPP_INFO(get_logger(), "sent set_pos -> (%.3f, %.3f, %.3f)", x, y, z);
}

void ManualControlNode::onGoalResult(const GoalHandle::WrappedResult & result)
{
  if (!result.result) return;
  if (result.result->status == SetPosition::Result::STATUS_UNREACHABLE) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "set_pos target was rejected as unreachable; the next key press starts from the "
                         "measured pose, so the target will not move further into that area");
  } else if (result.result->status == SetPosition::Result::STATUS_POS_NOT_REACHED) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "set_pos target was not reached; the next key press starts from the measured pose");
  }
}

void ManualControlNode::onPosition(const arm::msg::ArmPosition::SharedPtr msg)
{
  const double x = msg->position.x;
  const double y = msg->position.y;
  const double z = msg->position.z;

  pos_x_ = x;
  pos_y_ = y;
  pos_z_ = z;
  position_received_.store(true);

  const double tol = 1e-4;
  bool changed = !printed_once_ ||
                 std::fabs(x - last_x_) > tol ||
                 std::fabs(y - last_y_) > tol ||
                 std::fabs(z - last_z_) > tol;

  if (changed) {
    RCLCPP_INFO(get_logger(), "pos = (%.4f, %.4f, %.4f)", x, y, z);
    last_x_ = x;
    last_y_ = y;
    last_z_ = z;
    printed_once_ = true;
  }
}

}  // namespace DeltaArmRos