#ifndef arm__MOTOR_DRIVER_NODE_HPP_
#define arm__MOTOR_DRIVER_NODE_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "arm/msg/arm_feedback.hpp"
#include "arm/msg/motor_targets.hpp"
#include "arm/srv/motor_param_query.hpp"
#include "arm/motor_driver.hpp"

namespace DeltaArmDriverNode
{

/**
 * @brief Bridges ROS traffic to the physical FashionStart bus-servos.
 *
 *  * subscribes "arm/motor_targets"  -> applies joint angles to the motors
 *  * services  "arm/motor_param_query" -> generic hardware/param query
 *    (servo_id, request_type -> response_type, value)
 */
class MotorDriverNode : public rclcpp::Node
{
public:
  explicit MotorDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParams();
  void onMotorTargets(const arm::msg::MotorTargets::SharedPtr msg);
  void onParamQuery(
    const std::shared_ptr<arm::srv::MotorParamQuery::Request> request,
    const std::shared_ptr<arm::srv::MotorParamQuery::Response> response);
  void publishFeedback();
  void handleServoFault(int index, int error_code);

  std::shared_ptr<DeltaArmDriver::MotorDriver> driver_;
  rclcpp::Subscription<arm::msg::MotorTargets>::SharedPtr target_sub_;
  rclcpp::Service<arm::srv::MotorParamQuery>::SharedPtr query_srv_;
  rclcpp::Publisher<arm::msg::ArmFeedback>::SharedPtr feedback_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

  double feedback_rate_;
  std::string feedback_frame_;
  int8_t online_[3];
  int8_t error_[3];
};

}  // namespace DeltaArmDriverNode

#endif  // arm__MOTOR_DRIVER_NODE_HPP_