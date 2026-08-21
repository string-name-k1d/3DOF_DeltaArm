#ifndef arm_HARDWARE__DELTA_ARM_MANUAL_HPP_
#define arm_HARDWARE__DELTA_ARM_MANUAL_HPP_

#include <atomic>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm_msgs/action/set_position.hpp"
#include "arm_msgs/msg/arm_position.hpp"
#include "arm_msgs/srv/toggle_position_stream.hpp"

namespace DeltaArmRos
{

/**
 * @brief Manual (keyboard) control node.
 *
 *  * Action-client to "arm/set_pos": remote controls the arm.
 *  * Subscribes "arm/pos" and prints current-position changes.
 *  * Enables the get_pos stream on startup so position feedback is available.
 *
 * The actual WASD key decoding lives in the standalone main
 * (delta_arm_manual_main.cpp); this class exposes the ROS plumbing.
 */
class ManualControlNode : public rclcpp::Node
{
public:
  using SetPosition = arm_msgs::action::SetPosition;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SetPosition>;

  explicit ManualControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Fire-and-forget a task-space target; replaces any in-flight goal.
  void send_target(double x, double y, double z);

  /// @brief Ask the controller to stream "arm/pos" continuously.
  void enable_position_streaming();

  /// Last position estimate received from the controller.
  std::atomic<double> pos_x_;
  std::atomic<double> pos_y_;
  std::atomic<double> pos_z_;

private:
  void onPosition(const arm_msgs::msg::ArmPosition::SharedPtr msg);

  rclcpp_action::Client<SetPosition>::SharedPtr action_client_;
  rclcpp::Subscription<arm_msgs::msg::ArmPosition>::SharedPtr pos_sub_;
  rclcpp::Client<arm_msgs::srv::TogglePositionStream>::SharedPtr toggle_client_;

  std::atomic<double> last_x_;
  std::atomic<double> last_y_;
  std::atomic<double> last_z_;
  std::atomic<bool> printed_once_;
};

}  // namespace DeltaArmRos

#endif  // arm_HARDWARE__DELTA_ARM_MANUAL_HPP_