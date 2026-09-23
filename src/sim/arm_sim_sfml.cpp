#include "arm/arm_sim_sfml.hpp"

#include <algorithm>
#include <cmath>

namespace DeltaArmSim
{

ArmSimSFMLNode::ArmSimSFMLNode(const rclcpp::NodeOptions & options)
  : Node("arm_sim_sfml", options)
{
  action_client_ = rclcpp_action::create_client<SetPosition>(this, "arm/set_pos");

  pos_sub_ = create_subscription<arm::msg::ArmPosition>(
    "arm/pos", 10,
    [this](const arm::msg::ArmPosition::SharedPtr msg) { onPosition(msg); });

  fb_sub_ = create_subscription<arm::msg::ArmFeedback>(
    "arm/motor_feedback", 10,
    [this](const arm::msg::ArmFeedback::SharedPtr msg) { onFeedback(msg); });

  toggle_client_ = create_client<arm::srv::TogglePositionStream>("arm/get_pos");

  RCLCPP_INFO(get_logger(),
              "ArmSim SFML node started - sending arm/set_pos goals, drawing "
              "arm/pos feedback; real arm/motor_feedback overrides the view");
}

void ArmSimSFMLNode::send_target(double x, double y, double z)
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "set_pos action server not available yet");
    return;
  }

  auto goal = SetPosition::Goal();
  goal.target.linear.x = x;
  goal.target.linear.y = y;
  goal.target.linear.z = z;
  action_client_->async_send_goal(goal);
  RCLCPP_INFO(get_logger(), "sent set_pos -> (%.3f, %.3f, %.3f)", x, y, z);
}

void ArmSimSFMLNode::enable_position_streaming()
{
  if (streaming_) return;

  if (!toggle_client_->wait_for_service(std::chrono::seconds(3))) {
    RCLCPP_WARN(get_logger(), "get_pos service not available - cannot enable streaming");
    return;
  }

  auto req = std::make_shared<arm::srv::TogglePositionStream::Request>();
  req->enable = true;
  toggle_client_->async_send_request(req);
  streaming_ = true;
}

void ArmSimSFMLNode::onPosition(const arm::msg::ArmPosition::SharedPtr msg)
{
  pos_x_ = msg->position.x;
  pos_y_ = msg->position.y;
  pos_z_ = msg->position.z;
  for (int i = 0; i < 3; ++i) {
    ang_[i] = msg->motor_angles_current[i];
    ang_tar_[i] = msg->motor_angles_target[i];
  }
  got_pos_ = true;
}

void ArmSimSFMLNode::onFeedback(const arm::msg::ArmFeedback::SharedPtr msg)
{
  for (int i = 0; i < 3; ++i) {
    fb_ang_[i] = msg->servo_angle_current[i];
    fb_online_[i] = msg->servo_online[i];
    fb_error_[i] = msg->servo_error[i];
  }
  fb_last_ = now();
  got_fb_ = true;
}

bool ArmSimSFMLNode::feedback_live() const
{
  if (!got_fb_) return false;
  if ((now() - fb_last_).seconds() > 0.5) return false;
  for (int i = 0; i < 3; ++i) {
    if (fb_online_[i] != 1 || fb_ang_[i] < 0.0f) return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// Anti-shake: sticky source selection + smoothed display angles
// ---------------------------------------------------------------------------
bool ArmSimSFMLNode::select_source()
{
  const bool good = feedback_live();

  if (good) {
    fb_streak_ = std::min(fb_streak_ + 1, kFbAcquireStreak);
    fb_lost_ = now();
    fb_ever_good_ = true;
    if (!fb_active_ && fb_streak_ >= kFbAcquireStreak) {
      fb_active_ = true;  // acquire: require a consistent run, not one msg
    }
  } else {
    fb_streak_ = 0;
    if (fb_active_ && fb_ever_good_ &&
        (now() - fb_lost_).seconds() > kFbHoldSeconds) {
      fb_active_ = false;  // only drop after the hold period (no flicker)
    }
  }
  return fb_active_;
}

void ArmSimSFMLNode::update_display(const float raw[3], float dt_seconds)
{
  if (!display_init_) {
    for (int i = 0; i < 3; ++i) display_ang_[i] = raw[i];
    display_init_ = true;
    return;
  }

  // dt-based exponential smoothing (frame-rate independent). Jitter of ~1 deg
  // at 10-20 Hz message rates is attenuated ~10x; real motion still tracks.
  const double alpha = 1.0 - std::exp(-static_cast<double>(dt_seconds) / kDispTauSeconds);
  for (int i = 0; i < 3; ++i) {
    if (std::fabs(raw[i] - display_ang_[i]) > kDispSnapDeg) {
      display_ang_[i] = raw[i];  // big jump (mode switch / retarget): snap
    } else {
      display_ang_[i] = static_cast<float>(
        display_ang_[i] + alpha * (raw[i] - display_ang_[i]));
    }
  }
}

}  // namespace DeltaArmSim