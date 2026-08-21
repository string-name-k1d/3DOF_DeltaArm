#ifndef arm_HARDWARE__MOTOR_DRIVER_NODE_HPP_
#define arm_HARDWARE__MOTOR_DRIVER_NODE_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "arm_msgs/msg/motor_targets.hpp"
#include "arm_msgs/srv/motor_param_query.hpp"
#include "arm_hardware/motor_driver.hpp"

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
  void onMotorTargets(const arm_msgs::msg::MotorTargets::SharedPtr msg);
  void onParamQuery(
    const std::shared_ptr<arm_msgs::srv::MotorParamQuery::Request> request,
    const std::shared_ptr<arm_msgs::srv::MotorParamQuery::Response> response);

  std::shared_ptr<DeltaArmDriver::MotorDriver> driver_;
  rclcpp::Subscription<arm_msgs::msg::MotorTargets>::SharedPtr target_sub_;
  rclcpp::Service<arm_msgs::srv::MotorParamQuery>::SharedPtr query_srv_;
};

}  // namespace DeltaArmDriverNode

#endif  // arm_HARDWARE__MOTOR_DRIVER_NODE_HPP_