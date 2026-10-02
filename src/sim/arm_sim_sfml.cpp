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

  configureKinematics();

  RCLCPP_INFO(get_logger(),
              "ArmSim SFML node started - sending arm/set_pos goals, drawing "
              "arm/pos feedback; real arm/motor_feedback overrides the view");
}

// Read the shared geometry.* parameters (the same ones the controller reads) and
// hand them to a DeltaArm::Arm used purely as a kinematics reference, so the
// motor -> arm angle conversion this renderer draws is the controller's own
// code rather than a second implementation that can drift out of sync.
void ArmSimSFMLNode::configureKinematics()
{
  const auto base_radius = declare_parameter<double>("geometry.base_radius", 100.0);
  const auto platform_radius = declare_parameter<double>("geometry.platform_radius", 32.5);
  const auto upper_arm_len = declare_parameter<double>("geometry.upper_arm_len", 120.0);
  const auto lower_arm_len = declare_parameter<double>("geometry.lower_arm_len", 240.0);
  const auto servo_radius = declare_parameter<double>("geometry.servo_radius", 57.65);
  const auto servo_z = declare_parameter<double>("geometry.servo_z", -22.5);
  const auto upper_rod_len = declare_parameter<double>("geometry.upper_rod_len", 60.0);
  const auto servo_rod_len = declare_parameter<double>("geometry.servo_rod_len", 35.0);
  const auto arm_attach_dist = declare_parameter<double>("geometry.arm_attach_dist", 68.5);
  const auto arm_attach_offset = declare_parameter<double>("geometry.arm_attach_offset", 20.5);
  const auto angle_min = declare_parameter<std::vector<double>>(
    "geometry.angle_min", std::vector<double>{0.0, 0.0, 0.0});
  const auto angle_max = declare_parameter<std::vector<double>>(
    "geometry.angle_max", std::vector<double>{145.0, 145.0, 145.0});
  const auto arm_angle_min = declare_parameter<double>("geometry.arm_angle_min", 0.0);
  const auto arm_angle_max = declare_parameter<double>("geometry.arm_angle_max", 180.0);
  constexpr double kDegToRad = 0.017453292519943295;
  constexpr double kTwoPiOver3 = 2.0943951023931953;

  std::vector<DeltaArm::ArmMechConfig> configs;
  for (int leg = 0; leg < 3; ++leg) {
    const size_t idx = static_cast<size_t>(leg);
    DeltaArm::ArmMechConfig c{
      .base_radius = static_cast<float>(base_radius),
      .platform_radius = static_cast<float>(platform_radius),
      .upper_arm_len = static_cast<float>(upper_arm_len),
      .lower_arm_len = static_cast<float>(lower_arm_len),
      .servo_radius = static_cast<float>(servo_radius),
      .servo_z = static_cast<float>(servo_z),
      .upper_rod_len = static_cast<float>(upper_rod_len),
      .servo_rod_len = static_cast<float>(servo_rod_len),
      .arm_attach_dist = static_cast<float>(arm_attach_dist),
      .arm_attach_offset = static_cast<float>(arm_attach_offset),
      .motor_angle_min = static_cast<float>(idx < angle_min.size() ? angle_min[idx] : 0.0),
      .motor_angle_max = static_cast<float>(idx < angle_max.size() ? angle_max[idx] : 145.0),
      .arm_angle_min = static_cast<float>(arm_angle_min * kDegToRad),
      .arm_angle_max = static_cast<float>(arm_angle_max * kDegToRad),
      .plane_angle = static_cast<float>(leg * kTwoPiOver3),
      .home_offset = 0.0f,
    };
    configs.push_back(c);
  }
  kin_.set_geometry(configs);

  float lo_deg = 0.0f;
  float hi_deg = 0.0f;
  if (kin_.get_linkage_band(0, lo_deg, hi_deg)) {
    RCLCPP_INFO(get_logger(),
                "reachable arm band (shared with the controller): [%.2f, %.2f] deg", lo_deg, hi_deg);
  } else {
    RCLCPP_WARN(get_logger(),
                "reachable arm band is EMPTY - the 4-bar cannot close within the servo's travel");
  }
}

bool ArmSimSFMLNode::seed_pose(double x, double y, double z, float motor_out[3],
                               std::string & reason)
{
  const DeltaArm::TargetResult r =
    kin_.set_tar_pos(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
  if (!r.reached) {
    reason = r.reason;
    return false;
  }
  kin_.get_motor_targets(motor_out);
  return true;
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

  // Watch the result: an unreachable target is aborted by the controller, and
  // without this the keys could walk the target marker out of the envelope with
  // the arm simply refusing to move and nothing on screen to say why.
  rclcpp_action::Client<SetPosition>::SendGoalOptions opts;
  opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<SetPosition>::WrappedResult & result) {
      const auto & res = result.result;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        last_rejection_.clear();
        return;
      }
      // res is null for a cancelled/abandoned goal, so do not dereference it.
      last_rejection_ = (!res || res->message.empty()) ? "set_pos goal was not accepted" : res->message;
      RCLCPP_WARN(get_logger(), "set_pos goal REFUSED: %s", last_rejection_.c_str());
    };
  action_client_->async_send_goal(goal, opts);
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

  tar_x_ = msg->target_position.x;
  tar_y_ = msg->target_position.y;
  tar_z_ = msg->target_position.z;
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