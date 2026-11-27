#ifndef arm__MOTOR_DRIVER_NODE_HPP_
#define arm__MOTOR_DRIVER_NODE_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "arm/msg/motor_feedback.hpp"
#include "arm/msg/motor_targets.hpp"
#include "arm/motor_driver.hpp"

namespace DeltaArmDriverNode
{

/// ROS wrapper for the FashionStar UART motor driver.
class MotorDriverNode : public rclcpp::Node
{
public:
  explicit MotorDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParams();
  void onMotorTargets(const arm::msg::MotorTargets::SharedPtr msg);
  void publishFeedback();
  void handleServoFault(int index, int error_code);

  std::shared_ptr<DeltaArmDriver::MotorDriver> driver_;
  rclcpp::Subscription<arm::msg::MotorTargets>::SharedPtr target_sub_;
  rclcpp::Publisher<arm::msg::MotorFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  double feedback_rate_;
  std::string feedback_frame_;
  int8_t online_[3];
  int8_t error_[3];
  bool enable_motion_;


  std::string targets_topic_;


  int ping_fail_streak_[3] = {0, 0, 0};
  int query_fail_streak_[3] = {0, 0, 0};
  float last_good_angle_[3] = {-1.0f, -1.0f, -1.0f};
  int offline_streak_ = 3;


  std::mutex bus_mutex_;
};

}

#endif
