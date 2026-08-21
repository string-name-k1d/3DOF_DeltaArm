#include "arm_hardware/delta_arm_manual.hpp"

#include <cmath>

namespace DeltaArmRos
{

ManualControlNode::ManualControlNode(const rclcpp::NodeOptions & options)
  : Node("arm_manual_control", options),
    pos_x_(0.0), pos_y_(0.0), pos_z_(0.0),
    last_x_(0.0), last_y_(0.0), last_z_(0.0),
    printed_once_(false)
{
  action_client_ = rclcpp_action::create_client<SetPosition>(this, "arm/set_pos");

  pos_sub_ = create_subscription<arm_msgs::msg::ArmPosition>(
    "arm/pos", 10,
    [this](const arm_msgs::msg::ArmPosition::SharedPtr msg) { onPosition(msg); });

  toggle_client_ = create_client<arm_msgs::srv::TogglePositionStream>("arm/get_pos");

  RCLCPP_INFO(get_logger(), "arm_manual_control ready (waiting for set_pos server)");
}

void ManualControlNode::send_target(double x, double y, double z)
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

  // Fire-and-forget: the controller accepts & executes immediately, and a newer
  // goal simply updates the target.
  action_client_->async_send_goal(goal);

  RCLCPP_INFO(get_logger(), "sent set_pos -> (%.3f, %.3f, %.3f)", x, y, z);
}

void ManualControlNode::enable_position_streaming()
{
  if (!toggle_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_WARN(get_logger(), "get_pos service not available - cannot enable streaming");
    return;
  }

  auto req = std::make_shared<arm_msgs::srv::TogglePositionStream::Request>();
  req->enable = true;
  toggle_client_->async_send_request(req);
}

void ManualControlNode::onPosition(const arm_msgs::msg::ArmPosition::SharedPtr msg)
{
  const double x = msg->position.x;
  const double y = msg->position.y;
  const double z = msg->position.z;

  pos_x_ = x;
  pos_y_ = y;
  pos_z_ = z;

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