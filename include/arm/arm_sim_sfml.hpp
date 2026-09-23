#ifndef arm__ARM_SIM_SFML_HPP_
#define arm__ARM_SIM_SFML_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm/action/set_position.hpp"
#include "arm/msg/arm_feedback.hpp"
#include "arm/msg/arm_position.hpp"
#include "arm/srv/toggle_position_stream.hpp"

namespace DeltaArmSim
{

/**
 * @brief 2-D graphical (SFML) delta-arm simulator.
 *
 * Presents a live, self-contained view of the arm and forwards user targets to
 * the arm controller *exactly* like the manual-control node does:
 *
 *  * action-client to "arm/set_pos"  : forwards commanded targets;
 *  * subscribes "arm/pos"             : current position + motor angles;
 *  * enables "arm/get_pos" streaming  : so "arm/pos" is published.
 *
 * While a fresh, all-online "arm/motor_feedback" message is arriving, the
 * renderer instead draws the REAL machine: the three measured servo angles
 * (degrees, joint frame) drive the FK geometry, so the picture mirrors the
 * physical arm driven by the motor driver.
 *
 * The SFML renderer shows two simultaneous projections of the arm from the
 * live motor angles:
 *
 *  * top view   - the arm as seen looking down the base's vertical axis: the
 *                 three limbs project onto the XY plane;
 *  * side view  - the arm as seen looking along the Y axis: the XZ projection
 *                 of two representative limbs (upper arm + lower
 *                 parallelogram rod + end-effector platform), which shows the
 *                 full "side" of a delta arm.
 *
 * Both projections share a common pixel scale so movement looks consistent.
 */
class ArmSimSFMLNode : public rclcpp::Node
{
public:
  using SetPosition = arm::action::SetPosition;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SetPosition>;

  explicit ArmSimSFMLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Send a target; replaces any in-flight goal (mirrors the manual node).
  void send_target(double x, double y, double z);

  /// @brief Ask the controller for continuous "arm/pos" publishing.
  void enable_position_streaming();

  /// Latest end-effector position (m) from the controller stream.
  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double pos_z_ = 0.0;

  /// Latest motor angles (degrees) from the controller stream.
  float ang_[3] = {0.0f, 0.0f, 0.0f};

  /// Latest motor TARGET angles (degrees) from the controller stream
  /// (arm/pos.motor_angles_target) - shown alongside the current ones.
  float ang_tar_[3] = {0.0f, 0.0f, 0.0f};

  /// Whether streamed position feedback has arrived at least once.
  bool got_pos_ = false;

  /// Latest measured motor angles (degrees, joint frame) from the real driver.
  float fb_ang_[3] = {-1.0f, -1.0f, -1.0f};
  /// Servo online / error flags from the real driver.
  int fb_online_[3] = {0, 0, 0};
  int fb_error_[3] = {0, 0, 0};

  /// True when a fresh, all-online "arm/motor_feedback" message arrived
  /// recently; the renderer then draws the REAL machine instead of the sim
  /// stream (arm/pos).
  bool feedback_live() const;

  // ── Anti-shake: source hysteresis + last-good latch + smoothing ─────────
  //
  // The drawn motor angles previously flipped every frame between the real
  // servo readback (arm/motor_feedback) and the controller estimate (arm/pos)
  // whenever a single serial ping/query failed - the two disagree by the
  // servo install offsets, so the drawing visibly "shook" without any motion.
  // The selection is now sticky:
  //
  //  * acquiring real feedback requires kFbAcquireStreak consecutive good
  //    messages;
  //  * once acquired it is HELD for kFbHoldSeconds after the feedback drops
  //    out (drawing the last measured angles, not the estimate);
  //  * only after the hold expires does the view fall back to arm/pos.
  //
  // On top of the source selection the drawn angles are exponentially
  // smoothed (update_display) so servo encoder jitter does not vibrate the
  // picture; large jumps (> kDispSnapDeg) snap instead of lagging.
  static constexpr int    kFbAcquireStreak = 3;     // good msgs before switch
  static constexpr double kFbHoldSeconds   = 1.5;   // keep fb this long after loss
  static constexpr float  kDispSnapDeg     = 25.0f; // snap instead of glide past
  static constexpr double kDispTauSeconds  = 0.15;  // smoothing time constant

  /// Per-frame source selection with hysteresis. Returns true when the
  /// renderer should draw the REAL servo feedback, false for the sim stream.
  /// Call once per rendered frame BEFORE update_display().
  bool select_source();

  /// Feed the currently-selected raw motor angles + frame time (seconds).
  /// Smooths into display_ang_ (used by the renderer) with a dt-based EMA.
  void update_display(const float raw[3], float dt_seconds);

  /// Smoothed motor angles to draw (degrees).
  const float * display_angles() const { return display_ang_; }

  /// Which source the current display angles come from (post-hysteresis).
  bool feedback_source_active() const { return fb_active_; }

private:
  void onPosition(const arm::msg::ArmPosition::SharedPtr msg);
  void onFeedback(const arm::msg::ArmFeedback::SharedPtr msg);

  rclcpp_action::Client<SetPosition>::SharedPtr action_client_;
  rclcpp::Subscription<arm::msg::ArmPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<arm::msg::ArmFeedback>::SharedPtr fb_sub_;
  rclcpp::Client<arm::srv::TogglePositionStream>::SharedPtr toggle_client_;

  bool streaming_ = false;
  rclcpp::Time fb_last_;
  bool got_fb_ = false;

  // Anti-shake state (see comment above).
  bool fb_active_ = false;      // sticky source selection
  int  fb_streak_ = 0;          // consecutive good feedback messages
  bool fb_ever_good_ = false;   // feedback seen valid at least once
  rclcpp::Time fb_lost_;        // when feedback stopped being good
  float display_ang_[3] = {0.0f, 0.0f, 0.0f};
  bool display_init_ = false;
};

}  // namespace DeltaArmSim

#endif  // arm__ARM_SIM_SFML_HPP_