#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "arm/msg/arm_feedback.hpp"
#include "arm/msg/arm_health.hpp"
#include "arm/msg/control_state.hpp"

namespace DeltaArmRos {

class ArmHealthMonitorNode : public rclcpp::Node {
public:
  explicit   ArmHealthMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("arm_health_monitor", options) {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    publish_rate_ = declare_parameter<double>("publish_rate", 2.0);
    stale_timeout_sec_ = declare_parameter<double>("stale_timeout_sec", 2.0);
    require_endpoint_ = declare_parameter<bool>("require_endpoint", false);

    health_pub_ = create_publisher<arm::msg::ArmHealth>("arm/health", 10);

    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    control_state_sub_ = create_subscription<arm::msg::ControlState>(
      "arm/control_state", latched_qos, [this](const arm::msg::ControlState::SharedPtr msg) {
        last_control_state_ = msg;
        last_control_state_time_ = now();
      });

    motor_feedback_sub_ = create_subscription<arm::msg::ArmFeedback>(
      "arm/motor_feedback", 10, [this](const arm::msg::ArmFeedback::SharedPtr msg) {
        last_motor_feedback_ = msg;
        last_motor_feedback_time_ = now();
      });

    const auto period = std::chrono::milliseconds(static_cast<int64_t>(
      1000.0 / (publish_rate_ > 0.0 ? publish_rate_ : 1.0)));
    timer_ = create_wall_timer(period, [this]() { publish(); });

    RCLCPP_INFO(get_logger(), "arm_health_monitor ready (arm/health @ %.1f Hz)", publish_rate_);
  }

private:
  static float ageSec(const rclcpp::Time & now_time, const rclcpp::Time & then_time) {
    if (then_time.nanoseconds() == 0) {
      return -1.0f;
    }
    return static_cast<float>((now_time - then_time).seconds());
  }

  static bool hasNode(const std::vector<std::string> & names, const std::string & name) {
    return std::find(names.begin(), names.end(), name) != names.end();
  }

  void publish() {
    const auto stamp = now();
    arm::msg::ArmHealth msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.motor_count = 3;
    msg.control_state_age_sec = ageSec(stamp, last_control_state_time_);
    msg.motor_feedback_age_sec = ageSec(stamp, last_motor_feedback_time_);

    for (size_t i = 0; i < 3; ++i) {
      msg.servo_online[i] = -1;
      msg.servo_error[i] = -1;
    }

    const auto node_names = get_node_names();
    msg.controller_seen = hasNode(node_names, "arm_controller");
    msg.driver_seen = hasNode(node_names, "arm_motor_driver");
    // The endpoint module is owned by arm_controller now (no arm_endpoint node);
    // treat a live arm/endpoint/state publisher as the endpoint being present.
    msg.endpoint_seen = count_publishers("arm/endpoint/state") > 0;

    const auto topics = get_topic_names_and_types();
    bool has_set_pos = false;
    bool has_targets_sub = false;
    for (const auto & entry : topics) {
      if (entry.first == "arm/set_pos/_action/status" || entry.first == "arm/set_pos/_action/feedback") {
        has_set_pos = true;
      }
      if (entry.first == "arm/motor_targets") {
        has_targets_sub = !entry.second.empty();
      }
    }
    const bool has_control_state_pub = count_publishers("arm/control_state") > 0;
    const bool has_motor_feedback_pub = count_publishers("arm/motor_feedback") > 0;

    msg.control_state_seen = last_control_state_ &&
      msg.control_state_age_sec >= 0.0f && msg.control_state_age_sec <= stale_timeout_sec_;
    msg.motor_feedback_seen = last_motor_feedback_ &&
      msg.motor_feedback_age_sec >= 0.0f && msg.motor_feedback_age_sec <= stale_timeout_sec_;

    // Controller/driver nodes can appear briefly; topic freshness is the primary signal.
    if (last_control_state_ && msg.control_state_age_sec >= 0.0f && msg.control_state_age_sec <= stale_timeout_sec_) {
      msg.control_state_seen = true;
      msg.controller_seen = true;
      msg.control_state = last_control_state_->state;
      msg.emergency_active = last_control_state_->emergency_active;
      msg.accepts_goals = last_control_state_->accepts_goals;
    } else if (last_control_state_ && has_control_state_pub) {
      msg.controller_seen = true;
      msg.control_state = last_control_state_->state;
      msg.emergency_active = last_control_state_->emergency_active;
      msg.accepts_goals = last_control_state_->accepts_goals;
    }
    if (msg.motor_feedback_seen) {
      msg.driver_seen = true;
      msg.motors_online = 0;
      msg.motors_faulted = 0;
      for (size_t i = 0; i < 3; ++i) {
        msg.servo_online[i] = last_motor_feedback_->servo_online[i];
        msg.servo_error[i] = last_motor_feedback_->servo_error[i];
        if (msg.servo_online[i] == 1) {
          ++msg.motors_online;
        }
        if (msg.servo_error[i] != 0 || msg.servo_online[i] != 1) {
          ++msg.motors_faulted;
        }
      }
    }

    // "motion_enabled" means the controller/stack is configured to accept or
    // forward position-target control, not that goals are currently accepted
    // (an e-stop intentionally blocks acceptance).
    msg.motion_enabled = has_set_pos || has_targets_sub || has_control_state_pub || msg.controller_seen;

    std::vector<std::string> issues;
    const bool controller_expected = msg.controller_seen || has_control_state_pub || has_set_pos;
    const bool driver_expected = msg.driver_seen || has_motor_feedback_pub || has_targets_sub;

    if (!msg.control_state_seen) {
      if (!controller_expected) {
        issues.emplace_back("arm/control_state missing");
      } else if (!has_control_state_pub) {
        issues.emplace_back(last_control_state_ ? "arm/control_state stale" : "arm/control_state missing");
      } else if (!last_control_state_) {
        issues.emplace_back("arm/control_state not received yet");
      } else {
        issues.emplace_back("arm/control_state stale");
      }
    }
    if (!msg.motor_feedback_seen) {
      if (driver_expected) {
        issues.emplace_back(last_motor_feedback_ ? "arm/motor_feedback stale" : "arm/motor_feedback missing");
      } else if (!controller_expected) {
        issues.emplace_back("arm/motor_feedback missing");
      }
    }
    if (msg.emergency_active) {
      issues.emplace_back("emergency stop active");
    }
    if (msg.motor_feedback_seen) {
      if (msg.motors_online == 0) {
        issues.emplace_back("no motors online");
      } else if (msg.motors_online < msg.motor_count) {
        issues.emplace_back("one or more motors offline/faulted");
      }
    }
    if (require_endpoint_ && !msg.endpoint_seen) {
      issues.emplace_back("arm/endpoint/state missing");
    }

    msg.issues = issues;
    msg.healthy = issues.empty();
    msg.ready = msg.healthy && msg.accepts_goals && (!driver_expected || msg.motors_online == msg.motor_count);

    if (!msg.healthy) {
      const bool severe = msg.emergency_active || (controller_expected && !has_control_state_pub && !msg.control_state_seen) ||
                          (msg.motor_feedback_seen && msg.motors_online == 0);
      msg.status = severe ? "fault" : "degraded";
    } else {
      msg.status = "ok";
    }

    health_pub_->publish(msg);
  }

  std::string frame_id_;
  double publish_rate_{2.0};
  double stale_timeout_sec_{2.0};
  bool require_endpoint_{false};

  arm::msg::ControlState::SharedPtr last_control_state_;
  arm::msg::ArmFeedback::SharedPtr last_motor_feedback_;
  rclcpp::Time last_control_state_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_motor_feedback_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<arm::msg::ArmHealth>::SharedPtr health_pub_;
  rclcpp::Subscription<arm::msg::ControlState>::SharedPtr control_state_sub_;
  rclcpp::Subscription<arm::msg::ArmFeedback>::SharedPtr motor_feedback_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace DeltaArmRos

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DeltaArmRos::ArmHealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
