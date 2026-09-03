#include "arm/motor_driver_node.hpp"

#include <chrono>

namespace DeltaArmDriverNode
{

MotorDriverNode::MotorDriverNode(const rclcpp::NodeOptions & options)
  : Node("arm_motor_driver", options),
    feedback_rate_(10.0),
    feedback_frame_("base_link"),
    online_{0, 0, 0},
    error_{0, 0, 0}
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
      for (int i = 0; i < 3; ++i) {
        bool online = false;
        driver_->ping(i, &online);
        online_[i] = online ? 1 : 0;
        if (!online) handleServoFault(i, 2);
      }
    } else {
      RCLCPP_WARN(get_logger(),
                  "motor driver could not sync all servos on %s - is the adapter connected?",
                  port.c_str());
      for (int i = 0; i < 3; ++i) handleServoFault(i, 2);
    }
  }

  target_sub_ = create_subscription<arm::msg::MotorTargets>(
    "arm/motor_targets", 10,
    std::bind(&MotorDriverNode::onMotorTargets, this, std::placeholders::_1));

  query_srv_ = create_service<arm::srv::MotorParamQuery>(
    "arm/motor_param_query",
    std::bind(&MotorDriverNode::onParamQuery, this,
              std::placeholders::_1, std::placeholders::_2));

  feedback_pub_ = create_publisher<arm::msg::ArmFeedback>("arm/motor_feedback", 10);

  const auto period = std::chrono::milliseconds(
    static_cast<int64_t>(1000.0 / (feedback_rate_ > 0.0 ? feedback_rate_ : 1.0)));
  feedback_timer_ = create_wall_timer(period, [this]() { publishFeedback(); });

  RCLCPP_INFO(get_logger(), "motor_driver ready (targets: arm/motor_targets, "
                            "feedback: arm/motor_feedback, "
                            "query: arm/motor_param_query)");
}

void MotorDriverNode::declareParams()
{
  declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<std::vector<int64_t>>("servo_ids", {0, 1, 2});
  declare_parameter<std::vector<double>>("start_angles", {0.0, 0.0, 0.0});
  declare_parameter<bool>("auto_init", true);
  declare_parameter<double>("feedback_rate", 10.0);
  declare_parameter<std::string>("feedback_frame", "base_link");
}

void MotorDriverNode::onMotorTargets(const arm::msg::MotorTargets::SharedPtr msg)
{
  float angles[3] = {msg->angles[0], msg->angles[1], msg->angles[2]};
  driver_->set_target_angles(angles, 3);
}

void MotorDriverNode::onParamQuery(
  const std::shared_ptr<arm::srv::MotorParamQuery::Request> request,
  const std::shared_ptr<arm::srv::MotorParamQuery::Response> response)
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

void MotorDriverNode::handleServoFault(int index, int error_code)
{
  error_[index] = static_cast<int8_t>(error_code);
  online_[index] = 0;
  RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                       "servo %d FAULT code=%d (offline)", index, error_code);
}

void MotorDriverNode::publishFeedback()
{
  auto msg = std::make_shared<arm::msg::ArmFeedback>();
  msg->header.stamp = now();
  msg->header.frame_id = feedback_frame_;

  for (int i = 0; i < 3; ++i) {
    bool online = false;
    try {
      online = driver_->ping(i, &online) && online;
    } catch (...) {
      online = false;
    }
    online_[i] = online ? 1 : 0;

    if (online) {
      double angle = 0.0;
      try {
        angle = driver_->query_angle(i);
      } catch (...) {
        angle = -1.0;
      }
      msg->servo_angle_current[i] = static_cast<float>(angle == -1.0 ? -1.0 : angle);
      error_[i] = (angle == -1.0) ? 1 : 0;
    } else {
      msg->servo_angle_current[i] = -1.0f;
      if (error_[i] == 0) handleServoFault(i, 1);
    }
    msg->servo_online[i] = online_[i];
    msg->servo_error[i] = error_[i];
  }

  feedback_pub_->publish(*msg);
}

}  // namespace DeltaArmDriverNode