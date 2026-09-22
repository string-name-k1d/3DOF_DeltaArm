// arm_cmd_bridge - bridges the arm controller into the Gazebo Harmonic
// closed-loop simulation.
//
//   * subscribes "arm/motor_targets" (arm/msg/MotorTargets) - MOTOR angles in
//     DEGREES (single publication per accepted set_pos goal)
//   * maps them through the 4-bar linkage (motor -> arm) so the URDF shoulder
//     joints (which ARE the arm pivots) receive the correct arm angle
//   * publishes "/delta_arm/shoulder_<i>/cmd_pos" (std_msgs/Float64, RADIANS)
//     for the three JointPositionController plugins in the simulation.
//
// Publish an initial zero target shortly after startup so the position-servoed
// shoulders hold the home pose even before the first real command arrives.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <arm/msg/motor_targets.hpp>

namespace arm_gazebo
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.28318530717958647692;

// Locked-in mechanism constants (mirrors arm/config/arm_params.yaml). The
// servo drives the arm through a 4-bar linkage: horn (a) + rigid rod (b),
// shoulder->socket link c = hypot(attach_dist, offset), ground link
// d = servo->shoulder.
struct Linkage
{
  double base_radius = 100.0;
  double servo_radius = 57.65;
  double servo_z = -22.5;
  double horn = 60.0;
  double rod = 35.0;
  double attach_dist = 68.5;
  double attach_offset = 20.5;
  double c_link = 0.0;
  double d_link = 0.0;

  void update()
  {
    c_link = std::hypot(attach_dist, attach_offset);
    d_link = std::hypot(base_radius - servo_radius, -servo_z);
  }

  // Motor angle (rad, 0 = limb extended, growing = arm sweeping down) whose
  // closed-form arm-side angle equals armDeg (deg). 4-bar closed form from the
  // controller's ik_stage2.
  double motorRadFromArm(double armDeg) const
  {
    const double t = armDeg * kPi / 180.0;
    const double tb = kPi - t;  // 180 deg - arm angle
    const double A = 2.0 * horn * d_link * std::cos(tb) - 2.0 * rod * d_link;
    const double B = 2.0 * horn * d_link * std::sin(tb);
    const double C = c_link * c_link - horn * horn - rod * rod - d_link * d_link
                     + 2.0 * horn * rod * std::cos(tb);
    const double R = std::hypot(A, B);
    const double u = (R > 1e-9) ? C / R : 0.0;
    const double raw = std::atan2(B, A)
                       + std::acos(std::clamp(u, -1.0, 1.0));
    return kTwoPi - raw;
  }

  // Arm angle (radians) for a commanded motor angle (degrees). Monotone
  // bisection on the closing band; clamped to the nearest band edge.
  double armRadFromMotor(double motorDeg) const
  {
    // Lower edge: smallest arm (deg) at which the rod can still close,
    // |socket - servo| == horn + rod.
    double lo = 0.0, hi = 90.0;
    for (int i = 0; i < 48; ++i) {
      const double mid = 0.5 * (lo + hi);
      const double t = mid * kPi / 180.0;
      const double r = base_radius + attach_dist * std::cos(t)
                       - attach_offset * std::sin(t);
      const double zz = -attach_dist * std::sin(t) - attach_offset * std::cos(t);
      const double dist = std::hypot(r - servo_radius, zz - servo_z);
      if (dist <= horn + rod) hi = mid; else lo = mid;
    }
    const double band_lo = 0.5 * (lo + hi);
    const double band_hi = 90.0;

    const double m_lo = motorRadFromArm(band_lo);
    const double m_hi = motorRadFromArm(band_hi);
    const double motor_rad = motorDeg * kPi / 180.0;
    if (motor_rad <= m_lo) return band_lo * kPi / 180.0;
    if (motor_rad >= m_hi) return band_hi * kPi / 180.0;

    lo = band_lo;
    hi = band_hi;
    for (int i = 0; i < 60; ++i) {
      const double mid = 0.5 * (lo + hi);
      if (motorRadFromArm(mid) > motor_rad) hi = mid; else lo = mid;
    }
    return (0.5 * (lo + hi)) * kPi / 180.0;
  }
};

}  // namespace

class ArmCmdBridge : public rclcpp::Node
{
public:
  ArmCmdBridge()
  : Node("arm_cmd_bridge")
  {
    // Allow overriding the locked-in constants via params if needed.
    const double base_radius = declare_parameter<double>("geometry.base_radius", 100.0);
    const double servo_radius = declare_parameter<double>("geometry.servo_radius", 57.65);
    const double servo_z = declare_parameter<double>("geometry.servo_z", -22.5);
    const double horn = declare_parameter<double>("geometry.upper_rod_len", 60.0);
    const double rod = declare_parameter<double>("geometry.servo_rod_len", 35.0);
    const double attach = declare_parameter<double>("geometry.arm_attach_dist", 68.5);
    const double offset = declare_parameter<double>("geometry.arm_attach_offset", 20.5);
    linkage_ = Linkage{base_radius, servo_radius, servo_z, horn, rod, attach, offset, 0.0, 0.0};
    linkage_.update();

    for (int i = 0; i < 3; ++i) {
      pubs_[i] = create_publisher<std_msgs::msg::Float64>(
        "/delta_arm/shoulder_" + std::to_string(i) + "/cmd_pos", 10);
    }

    sub_ = create_subscription<arm::msg::MotorTargets>(
      "arm/motor_targets", rclcpp::SensorDataQoS(),
      [this](const arm::msg::MotorTargets::ConstSharedPtr msg) { onMotors(msg); });

    // One-shot: settle the shoulders at the home pose (arm = fully extended,
    // motor 0 -> band edge) at startup.
    home_timer_ = create_wall_timer(
      std::chrono::milliseconds(500), [this]() {
        publishCmd(0.0, 0.0, 0.0);
        home_timer_->cancel();
      });

    RCLCPP_INFO(get_logger(),
                "arm_cmd_bridge ready -> /delta_arm/shoulder_<i>/cmd_pos (rad, 4-bar motor->arm)");
  }

private:
  void onMotors(const arm::msg::MotorTargets::ConstSharedPtr msg)
  {
    const size_t n = msg->angles.size();
    publishCmd(
      (n > 0) ? msg->angles[0] : 0.0,
      (n > 1) ? msg->angles[1] : 0.0,
      (n > 2) ? msg->angles[2] : 0.0);
    RCLCPP_INFO(get_logger(), "motor targets (deg): %.3f %.3f %.3f",
                (n > 0) ? msg->angles[0] : 0.0,
                (n > 1) ? msg->angles[1] : 0.0,
                (n > 2) ? msg->angles[2] : 0.0);
  }

  void publishCmd(double m0, double m1, double m2)
  {
    const double rad[3] = {
      linkage_.armRadFromMotor(m0),
      linkage_.armRadFromMotor(m1),
      linkage_.armRadFromMotor(m2)};
    for (int i = 0; i < 3; ++i) {
      auto out = std::make_unique<std_msgs::msg::Float64>();
      out->data = rad[i];
      pubs_[i]->publish(std::move(out));
    }
  }

  Linkage linkage_;
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