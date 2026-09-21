// arm_cmd_bridge - bridges the arm controller into the Gazebo Harmonic
// closed-loop simulation.
//
//   * subscribes "arm/motor_targets" (arm/msg/MotorTargets) - shoulder joint
//     angles in DEGREES (single publication per accepted set_pos goal)
//   * publishes "/delta_arm/shoulder_<i>/cmd_pos" (std_msgs/Float64, RADIANS)
//     for the three JointPositionController plugins in the simulation.
//
// Publish an initial zero target shortly after startup so the position-servoed
// shoulders hold the home pose even before the first real command arrives.

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <arm/msg/motor_targets.hpp>

namespace arm_gazebo
{

class ArmCmdBridge : public rclcpp::Node
{
public:
  ArmCmdBridge()
  : Node("arm_cmd_bridge")
  {
    for (int i = 0; i < 3; ++i) {
      pubs_[i] = create_publisher<std_msgs::msg::Float64>(
        "/delta_arm/shoulder_" + std::to_string(i) + "/cmd_pos", 10);
    }

    sub_ = create_subscription<arm::msg::MotorTargets>(
      "arm/motor_targets", rclcpp::SensorDataQoS(),
      [this](const arm::msg::MotorTargets::ConstSharedPtr msg) { onMotors(msg); });

    // One-shot: settle the shoulders at the home pose (q = 0) at startup.
    home_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {
        publishCmd(0.0, 0.0, 0.0);
        home_timer_->cancel();
      });

    RCLCPP_INFO(get_logger(),
                "arm_cmd_bridge ready -> /delta_arm/shoulder_<i>/cmd_pos (rad)");
  }

private:
  void onMotors(const arm::msg::MotorTargets::ConstSharedPtr msg)
  {
    const size_t n = msg->angles.size();
    publishCmd(
      (n > 0) ? msg->angles[0] : 0.0,
      (n > 1) ? msg->angles[1] : 0.0,
      (n > 2) ? msg->angles[2] : 0.0);
    RCLCPP_INFO(get_logger(), "motor targets (deg -> rad): %.3f %.3f %.3f",
                (n > 0) ? msg->angles[0] : 0.0,
                (n > 1) ? msg->angles[1] : 0.0,
                (n > 2) ? msg->angles[2] : 0.0);
  }

  void publishCmd(double a0, double a1, double a2)
  {
    constexpr double kDegToRad = 0.017453292519943295;
    const double rad[3] = {a0 * kDegToRad, a1 * kDegToRad, a2 * kDegToRad};
    for (int i = 0; i < 3; ++i) {
      auto out = std::make_unique<std_msgs::msg::Float64>();
      out->data = rad[i];
      pubs_[i]->publish(std::move(out));
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubs_[3];
  rclcpp::Subscription<arm::msg::MotorTargets>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr home_timer_;
};

}  // namespace arm_gazebo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_gazebo::ArmCmdBridge>());
  rclcpp::shutdown();
  return 0;
}