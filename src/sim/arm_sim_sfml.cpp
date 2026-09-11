#include "arm/arm_sim_sfml.hpp"

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

  toggle_client_ = create_client<arm::srv::TogglePositionStream>("arm/get_pos");

  RCLCPP_INFO(get_logger(),
              "ArmSim SFML node started - sending arm/set_pos goals and "
              "drawing arm/pos feedback");
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
  }
  got_pos_ = true;
}

}  // namespace DeltaArmSim