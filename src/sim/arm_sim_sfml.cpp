#include "arm/arm_sim_sfml.hpp"

#include <cmath>
#include <chrono>

namespace DeltaArmSim
{

ArmSimSFMLNode::ArmSimSFMLNode(const rclcpp::NodeOptions & options)
  : Node("arm_sim_sfml", options),
    sim_x_(0.0f), sim_y_(0.0f), sim_z_(-0.30f),
    tar_x_(0.0f), tar_y_(0.0f), tar_z_(-0.30f),
    move_speed_(0.20f),
    cur_x_(0.0f), cur_y_(0.0f)
{
  servo_angles_.assign(3, 0.0f);

  cur_pub_ = create_publisher<geometry_msgs::msg::Point>("arm/current_position", 10);
  tar_sub_ = create_subscription<geometry_msgs::msg::Point>(
    "arm/target_position", 10,
    [this](const geometry_msgs::msg::Point::SharedPtr msg) { onTarget(msg); });

  // The ROS timer calls step(); each step also publishes current_position.
  timer_ = create_wall_timer(
    std::chrono::milliseconds(16),
    [this]() { step(0.016); });

  RCLCPP_INFO(get_logger(),
              "ArmSim SFML node started - publish to arm/target_position to move the arm");
}

void ArmSimSFMLNode::onTarget(const geometry_msgs::msg::Point::SharedPtr msg)
{
  tar_x_ = static_cast<float>(msg->x);
  tar_y_ = static_cast<float>(msg->y);
  tar_z_ = static_cast<float>(msg->z);
  RCLCPP_INFO(get_logger(), "target set to (%.3f, %.3f, %.3f)", tar_x_, tar_y_, tar_z_);
}

void ArmSimSFMLNode::step(double dt)
{
  // 1. Move the end-effector toward the target in Cartesian space.
  const float dx = tar_x_ - sim_x_;
  const float dy = tar_y_ - sim_y_;
  const float dz = tar_z_ - sim_z_;
  const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  const float step = move_speed_ * static_cast<float>(dt);
  if (dist <= step) {
    sim_x_ = tar_x_;
    sim_y_ = tar_y_;
    sim_z_ = tar_z_;
  } else {
    sim_x_ += dx / dist * step;
    sim_y_ += dy / dist * step;
    sim_z_ += dz / dist * step;
  }

  // 2. Solve the inverse kinematics and keep the last angles if unreachable.
  float targets[3] = {0.0f, 0.0f, 0.0f};
  arm_.set_tar_pos(sim_x_, sim_y_, sim_z_);
  arm_.get_motor_targets(targets);
  for (int i = 0; i < 3; ++i) {
    servo_angles_[i] = targets[i];
  }
  arm_.apply();
  const DeltaArm::Vec3 cur = arm_.get_cur_pos();

  // 3. Publish the current position (also mirrored into screen coords).
  auto msg = std::make_unique<geometry_msgs::msg::Point>();
  msg->x = cur.x;
  msg->y = cur.y;
  msg->z = cur.z;
  cur_pub_->publish(std::move(msg));

  // Screen-space: 2D side projection (x out of the page is folded into y).
  cur_x_ = cur.x * 1000.0f;
  cur_y_ = cur.z * -1000.0f;
}

}  // namespace DeltaArmSim
