#include "arm_hardware/motor_driver_node.hpp"

namespace DeltaArmDriverNode
{

MotorDriverNode::MotorDriverNode(const rclcpp::NodeOptions & options)
  : Node("arm_motor_driver", options)
{
  declareParams();

  const std::string port = get_parameter("port_name").as_string();
  const int baudrate = get_parameter("baudrate").as_int();
  const auto servo_ids = get_parameter("servo_ids").as_integer_array();
  const bool auto_init = get_parameter("auto_init").as_bool();

  int ids[3] = {0, 0, 0};
  for (size_t i = 0; i < servo_ids.size() && i < 3; ++i) ids[i] = static_cast<int>(servo_ids[i]);

  driver_ = std::make_shared<DeltaArmDriver::MotorDriver>(port, baudrate, ids);

  if (auto_init) {
    if (driver_->init()) {
      RCLCPP_INFO(get_logger(), "motor driver initialized on %s (%d baud)", port.c_str(), baudrate);
    } else {
      RCLCPP_WARN(get_logger(),
                  "motor driver could not sync all servos on %s - is the adapter connected?",
                  port.c_str());
    }
  }

  target_sub_ = create_subscription<arm_msgs::msg::MotorTargets>(
    "arm/motor_targets", 10,
    std::bind(&MotorDriverNode::onMotorTargets, this, std::placeholders::_1));

  query_srv_ = create_service<arm_msgs::srv::MotorParamQuery>(
    "arm/motor_param_query",
    std::bind(&MotorDriverNode::onParamQuery, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "motor_driver ready (targets: arm/motor_targets, "
                            "query: arm/motor_param_query)");
}

void MotorDriverNode::declareParams()
{
  declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<std::vector<int64_t>>("servo_ids", {0, 1, 2});
  declare_parameter<std::vector<double>>("start_angles", {0.0, 0.0, 0.0});
  declare_parameter<bool>("auto_init", true);
}

void MotorDriverNode::onMotorTargets(const arm_msgs::msg::MotorTargets::SharedPtr msg)
{
  float angles[3] = {msg->angles[0], msg->angles[1], msg->angles[2]};
  driver_->set_target_angles(angles, 3);
}

void MotorDriverNode::onParamQuery(
  const std::shared_ptr<arm_msgs::srv::MotorParamQuery::Request> request,
  const std::shared_ptr<arm_msgs::srv::MotorParamQuery::Response> response)
{
  response->response_type = request->request_type;
  response->value = 0;
  response->success = driver_->query(
    request->servo_id, request->request_type,
    &response->response_type, &response->value);

  RCLCPP_DEBUG(get_logger(), "query servo=%d type=%d -> response_type=%d value=%d success=%d",
               request->servo_id, request->request_type,
               response->response_type, response->value, int(response->success));
}

}  // namespace DeltaArmDriverNode