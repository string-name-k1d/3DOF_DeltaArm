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
};

}  // namespace DeltaArmSim

#endif  // arm__ARM_SIM_SFML_HPP_