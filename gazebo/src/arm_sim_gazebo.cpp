#include "arm_gazebo/arm_sim_gazebo.hpp"

#include <cmath>
#include <chrono>

namespace ArmGazebo
{

ArmSimGazeboNode::ArmSimGazeboNode(const rclcpp::NodeOptions & options)
  : Node("arm_sim_gazebo", options),
    sim_(0.0f, 0.0f, -0.30f),
    tar_(0.0f, 0.0f, -0.30f),
    cur_(0.0f, 0.0f, -0.30f),
    move_speed_(0.20f)
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
              "ArmSim Gazebo node started - publish to arm/target_position to move the arm");
}

void ArmSimGazeboNode::onTarget(const geometry_msgs::msg::Point::SharedPtr msg)
{
  tar_ = glm::vec3(static_cast<float>(msg->x),
                   static_cast<float>(msg->y),
                   static_cast<float>(msg->z));
  RCLCPP_INFO(get_logger(), "target set to (%.3f, %.3f, %.3f)", tar_.x, tar_.y, tar_.z);
}

void ArmSimGazeboNode::step(double dt)
{
  // 1. Move the end-effector toward the target in Cartesian space.
  const glm::vec3 delta = tar_ - sim_;
  const float dist = std::sqrt(glm::dot(delta, delta));
  const float step = move_speed_ * static_cast<float>(dt);
  if (dist <= step) {
    sim_ = tar_;
  } else {
    sim_ += delta / dist * step;
  }

  // 2. Solve the inverse kinematics and keep the last angles if unreachable.
  float targets[3] = {0.0f, 0.0f, 0.0f};
  arm_.set_tar_pos(sim_.x, sim_.y, sim_.z);
  arm_.get_motor_targets(targets);
  for (int i = 0; i < 3; ++i) {
    servo_angles_[i] = targets[i];
  }
  arm_.apply();
  const DeltaArm::Vec3 cur = arm_.get_cur_pos();
  cur_ = glm::vec3(cur.x, cur.y, cur.z);

  // 3. Publish the current position.
  auto msg = std::make_unique<geometry_msgs::msg::Point>();
  msg->x = cur_.x;
  msg->y = cur_.y;
  msg->z = cur_.z;
  cur_pub_->publish(std::move(msg));
}

}  // namespace ArmGazebo
