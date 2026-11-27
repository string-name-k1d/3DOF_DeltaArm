#ifndef arm__ARM_SIM_SFML_HPP_
#define arm__ARM_SIM_SFML_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "arm/action/set_position.hpp"
#include "arm/delta_arm.hpp"
#include "arm/msg/arm_position.hpp"
#include "arm/msg/motor_feedback.hpp"

namespace DeltaArmSim
{

/// Optional 2-D SFML visualiser for arm/pos and live motor feedback.
class ArmSimSFMLNode : public rclcpp::Node
{
public:
  using SetPosition = arm::action::SetPosition;
  using GoalHandle = rclcpp_action::ClientGoalHandle<SetPosition>;

  explicit ArmSimSFMLNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());


  void send_target(double x, double y, double z);


  float arm_angle_deg(float motor_deg, int leg) const
  {
    return kin_.arm_angle_from_motor_deg(motor_deg, leg);
  }


  bool arm_band(int leg, float & lo_deg, float & hi_deg) const
  {
    return kin_.get_linkage_band(leg, lo_deg, hi_deg);
  }


  bool try_arm_angle_deg(float motor_deg, int leg, float & arm_out) const
  {
    return kin_.try_arm_from_motor_deg(motor_deg, leg, arm_out);
  }


  bool arm_motor_span(int leg, float & motor_lo_deg, float & motor_hi_deg) const
  {
    return kin_.linkage_motor_span(leg, motor_lo_deg, motor_hi_deg);
  }


  bool seed_pose(double x, double y, double z, float motor_out[3], std::string & reason);


  const std::string & last_rejection() const { return last_rejection_; }
  bool goal_rejected() const { return !last_rejection_.empty(); }


  double pos_x_ = 0.0;
  double pos_y_ = 0.0;
  double pos_z_ = 0.0;


  double tar_x_ = 0.0;
  double tar_y_ = 0.0;
  double tar_z_ = 0.0;


  float ang_[3] = {0.0f, 0.0f, 0.0f};


  float ang_tar_[3] = {0.0f, 0.0f, 0.0f};


  bool got_pos_ = false;


  float fb_ang_[3] = {-1.0f, -1.0f, -1.0f};

  int fb_online_[3] = {0, 0, 0};
  int fb_error_[3] = {0, 0, 0};


  bool feedback_live() const;


  static constexpr int    kFbAcquireStreak = 3;
  static constexpr double kFbHoldSeconds   = 1.5;
  static constexpr float  kDispSnapDeg     = 25.0f;
  static constexpr double kDispTauSeconds  = 0.15;


  bool select_source();


  void update_display(const float raw[3], float dt_seconds);


  const float * display_angles() const { return display_ang_; }


  bool feedback_source_active() const { return fb_active_; }

private:
  void onPosition(const arm::msg::ArmPosition::SharedPtr msg);
  void onFeedback(const arm::msg::MotorFeedback::SharedPtr msg);


  void configureKinematics();

  rclcpp_action::Client<SetPosition>::SharedPtr action_client_;
  rclcpp::Subscription<arm::msg::ArmPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<arm::msg::MotorFeedback>::SharedPtr fb_sub_;


  DeltaArm::Arm kin_;

  std::string last_rejection_;

  rclcpp::Time fb_last_;
  bool got_fb_ = false;


  bool fb_active_ = false;
  int  fb_streak_ = 0;
  bool fb_ever_good_ = false;
  rclcpp::Time fb_lost_;
  float display_ang_[3] = {0.0f, 0.0f, 0.0f};
  bool display_init_ = false;
};

}

#endif