#include "arm/motor_driver_node.hpp"

#include <unistd.h>

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
  enable_motion_ = get_parameter("enable_motion").as_bool();
  targets_topic_ = get_parameter("targets_topic").as_string();
  offline_streak_ = get_parameter("offline_streak").as_int();
  const auto angle_min = get_parameter("angle_min").as_double_array();
  const auto angle_max = get_parameter("angle_max").as_double_array();
  const auto start_angles = get_parameter("start_angles").as_double_array();
  const double max_speed = get_parameter("max_speed").as_double();

  int ids[3] = {0, 0, 0};
  for (size_t i = 0; i < servo_ids.size() && i < 3; ++i) ids[i] = static_cast<int>(servo_ids[i]);

  DeltaArmDriver::MotorConfig config;
  for (size_t i = 0; i < 3; ++i) {
    config.angle_min[i] = (i < angle_min.size()) ? static_cast<float>(angle_min[i]) : 0.0f;
    config.angle_max[i] = (i < angle_max.size()) ? static_cast<float>(angle_max[i]) : 145.0f;
    config.install_offset[i] = (i < start_angles.size())
                                 ? static_cast<float>(start_angles[i]) : 0.0f;
  }
  config.max_speed = static_cast<float>(max_speed);

  // Pre-flight the serial device. The FashionStar SDK calls exit(-1) if the
  // port cannot be opened (FashionStar_UartServoProtocol.cpp), so avoid
  // constructing it when the device is missing - degrade to offline feedback.
  const bool port_ok = (access(port.c_str(), F_OK | R_OK | W_OK) == 0);
  if (!port_ok) {
    RCLCPP_WARN(get_logger(),
                "serial device %s not present/accessible - continuing in "
                "feedback-only mode with all servos reported offline. "
                "Connect the bus-servo adapter and relaunch for live feedback.",
                port.c_str());
    driver_ = nullptr;
  } else {
    driver_ = std::make_shared<DeltaArmDriver::MotorDriver>(port, baudrate, ids, config);
  }

  if (driver_ && auto_init) {
    bool ok = false;
    try {
      ok = driver_->init();
    } catch (...) {
      ok = false;
    }
    if (ok) {
      RCLCPP_INFO(get_logger(), "motor driver initialized on %s (%d baud)", port.c_str(), baudrate);
      for (int i = 0; i < 3; ++i) {
        bool online = false;
        try {
          driver_->ping(i, &online);
        } catch (...) {
          online = false;
        }
        online_[i] = online ? 1 : 0;
        if (!online) handleServoFault(i, 2);
      }
    } else {
      RCLCPP_WARN(get_logger(),
                  "motor driver could not sync all servos on %s - is the adapter connected?",
                  port.c_str());
      for (int i = 0; i < 3; ++i) handleServoFault(i, 2);
      RCLCPP_WARN(get_logger(),
                  "continuing in feedback-only mode (servos reported offline)");
    }
  }

  if (enable_motion_) {
    target_sub_ = create_subscription<arm::msg::MotorTargets>(
      targets_topic_, 10,
      std::bind(&MotorDriverNode::onMotorTargets, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(get_logger(),
                "motion control DISABLED (enable_motion=false) - "
                "%s subscription not started", targets_topic_.c_str());
  }

  query_srv_ = create_service<arm::srv::MotorParamQuery>(
    "arm/motor_param_query",
    std::bind(&MotorDriverNode::onParamQuery, this,
              std::placeholders::_1, std::placeholders::_2));

  feedback_pub_ = create_publisher<arm::msg::ArmFeedback>("arm/motor_feedback", 10);

  const auto period = std::chrono::milliseconds(
    static_cast<int64_t>(1000.0 / (feedback_rate_ > 0.0 ? feedback_rate_ : 1.0)));
  feedback_timer_ = create_wall_timer(period, [this]() { publishFeedback(); });

  RCLCPP_INFO(get_logger(), "motor_driver ready (targets: %s, "
                            "feedback: arm/motor_feedback, "
                            "query: arm/motor_param_query)",
              targets_topic_.c_str());
}

void MotorDriverNode::declareParams()
{
  declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
  declare_parameter<int>("baudrate", 115200);
  declare_parameter<std::vector<int64_t>>("servo_ids", {0, 1, 2});
  // Installation offset (deg) per motor: the physical servo angle when the
  // joint is at the arm's zero/home pose. Used to map joint-space commands
  // to physical servo angles and read-backs back to joint space.
  declare_parameter<std::vector<double>>("start_angles", {0.0, 0.0, 0.0});
  // Physical position limits (deg): 0 = fully extended, 145 = most retracted.
  declare_parameter<std::vector<double>>("angle_min", {0.0, 0.0, 0.0});
  declare_parameter<std::vector<double>>("angle_max", {145.0, 145.0, 145.0});
  // Velocity limit (deg/s) used for the on-servo trajectory profiling.
  declare_parameter<double>("max_speed", 100.0);
  declare_parameter<bool>("auto_init", true);
  declare_parameter<bool>("enable_motion", true);
  declare_parameter<double>("feedback_rate", 10.0);
  declare_parameter<std::string>("feedback_frame", "base_link");

  // Input target topic. The controller publishes its IK result (already run
  // through the control augmenter - offsets/feedforward/clamp are applied by
  // publishTargets(), see control_augmenter.hpp) on "arm/motor_targets"; the
  // driver consumes that stream directly. The parameter is kept for external
  // controllers that publish elsewhere.
  declare_parameter<std::string>("targets_topic", "arm/motor_targets");
  // Consecutive failed pings/queries before a servo is declared offline
  // (feedback debounce, see publishFeedback()).
  declare_parameter<int>("offline_streak", 3);
}

void MotorDriverNode::onMotorTargets(const arm::msg::MotorTargets::SharedPtr msg)
{
  float angles[3] = {msg->angles[0], msg->angles[1], msg->angles[2]};
  std::lock_guard<std::mutex> lock(bus_mutex_);
  driver_->set_target_angles(angles, 3);
}

void MotorDriverNode::onParamQuery(
  const std::shared_ptr<arm::srv::MotorParamQuery::Request> request,
  const std::shared_ptr<arm::srv::MotorParamQuery::Response> response)
{
  response->response_type = request->request_type;
  response->value = 0;
  if (!driver_) {
    response->success = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "motor param query ignored - no serial device open");
    return;
  }
  std::lock_guard<std::mutex> lock(bus_mutex_);
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

  if (!driver_) {
    for (int i = 0; i < 3; ++i) {
      online_[i] = 0;
      error_[i] = 2;  // offline (no serial device)
      msg->servo_online[i] = online_[i];
      msg->servo_error[i] = error_[i];
      msg->servo_angle_current[i] = -1.0f;
    }
    feedback_pub_->publish(*msg);
    return;
  }

  std::lock_guard<std::mutex> lock(bus_mutex_);
  for (int i = 0; i < 3; ++i) {
    // ── ping (debounced) ────────────────────────────────────────────────
    // A single dropped ping must NOT flip the servo offline: consumers that
    // switch between measured feedback and a model estimate (the 2-D
    // visualiser) would then visibly shake. Only `offline_streak_`
    // CONSECUTIVE failures declare the servo offline.
    bool ping_ok = false;
    try {
      bool reported_online = false;
      ping_ok = driver_->ping(i, &reported_online) && reported_online;
    } catch (...) {
      ping_ok = false;
    }

    if (ping_ok) {
      ping_fail_streak_[i] = 0;
      if (online_[i] != 1) {
        RCLCPP_INFO(get_logger(), "servo %d back online", i);
      }
      online_[i] = 1;
      error_[i] = 0;
    } else if (++ping_fail_streak_[i] >= offline_streak_) {
      if (error_[i] == 0) handleServoFault(i, 1);
      online_[i] = 0;
    }

    // ── angle read-back (debounced, holds the last good value) ──────────
    float angle = -1.0f;
    if (online_[i] == 1) {
      double queried = -1.0;
      try {
        queried = driver_->query_angle(i);
      } catch (...) {
        queried = -1.0;
      }

      if (queried != -1.0) {
        query_fail_streak_[i] = 0;
        angle = static_cast<float>(queried);
        last_good_angle_[i] = angle;  // cache for transient failures
      } else if (++query_fail_streak_[i] < offline_streak_ &&
                 last_good_angle_[i] >= 0.0f) {
        angle = last_good_angle_[i];  // hold last good - no -1 blip
      } else {
        angle = -1.0f;
        if (error_[i] == 0) handleServoFault(i, 1);
      }
    }

    msg->servo_angle_current[i] = angle;
    msg->servo_online[i] = online_[i];
    msg->servo_error[i] = error_[i];
  }

  feedback_pub_->publish(*msg);
}

}  // namespace DeltaArmDriverNode