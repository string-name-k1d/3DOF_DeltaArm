#include "arm/arm_endpoint_node.hpp"

#include <chrono>
#include <functional>
#include <utility>

namespace DeltaArmRos {

ArmEndpointNode::ArmEndpointNode(const rclcpp::NodeOptions& options) : Node("arm_endpoint", options) {
    declareParams();
    setupPublishers();
    setupSubscriptions();
    setupServices();
    setupTimer();

    RCLCPP_INFO(get_logger(), "arm_endpoint ready (module=%s, rate=%.1f Hz)", module_->type().c_str(),
                publish_rate_);
}

ArmEndpointNode::~ArmEndpointNode() { timer_->cancel(); }

void ArmEndpointNode::declareParams() {
    frame_id_ = declare_parameter<std::string>("frame_id", "base_link");
    publish_rate_ = declare_parameter<double>("publish_rate", 5.0);

    // Select the terminal module. Only "null" exists today; the factory falls
    // back to it so a gripper/camera value degrades instead of crashing.
    const std::string module_type = declare_parameter<std::string>("module_type", "null");
    module_ = EndpointModule::makeEndpointModule(module_type, *this);
    RCLCPP_INFO(get_logger(), "endpoint module: %s (active=%s)", module_->type().c_str(),
                module_->isActive() ? "yes" : "stub");
}

void ArmEndpointNode::setupPublishers() {
    state_pub_ = create_publisher<arm::msg::EndpointState>("arm/endpoint/state", 10);
    cog_pub_ = create_publisher<arm::msg::CogEstimate>("arm/endpoint/cog_estimate", 10);
    torque_pub_ = create_publisher<arm::msg::TorqueEstimate>("arm/endpoint/torque_estimate", 10);
}

void ArmEndpointNode::setupSubscriptions() {
    // Optional: reflect the controller's control state. Today it only records
    // transitions; a future version can gate CoG/torque publication on it.
    control_state_sub_ = create_subscription<arm::msg::ControlState>(
        "arm/control_state", 10, [this](const arm::msg::ControlState::SharedPtr msg) {
            if (msg->state != last_control_state_) {
                last_control_state_ = msg->state;
                RCLCPP_INFO(get_logger(), "arm control state -> %s (%s)", msg->state.c_str(),
                            msg->last_reason.c_str());
            }
        });
}

void ArmEndpointNode::setupServices() {
    // Toggle the dummy connector / attach-detach a payload. `data: true` attaches.
    attach_srv_ = create_service<std_srvs::srv::SetBool>(
        "arm/endpoint/attach", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> resp) {
            module_->setConnectorConnected(req->data);
            resp->success = true;
            resp->message = module_->connectorConnected() ? "payload attached" : "payload detached";
            RCLCPP_INFO(get_logger(), "%s (connector_connected=%s)", resp->message.c_str(),
                        module_->connectorConnected() ? "true" : "false");
            // Republish now so the effect is visible without waiting a cycle.
            publishOnce();
        });
}

void ArmEndpointNode::setupTimer() {
    const auto period =
        std::chrono::milliseconds(static_cast<int64_t>(1000.0 / (publish_rate_ > 0.0 ? publish_rate_ : 1.0)));
    timer_ = create_wall_timer(period, [this]() { publishOnce(); });
    publishOnce();
}

void ArmEndpointNode::publishOnce() {
    const auto stamp = now();

    arm::msg::EndpointState state_msg;
    state_msg.header.stamp = stamp;
    state_msg.header.frame_id = frame_id_;
    module_->fillEndpointState(state_msg);
    state_pub_->publish(state_msg);

    arm::msg::CogEstimate cog_msg;
    cog_msg.header.stamp = stamp;
    cog_msg.header.frame_id = frame_id_;
    module_->fillCogEstimate(cog_msg);
    cog_pub_->publish(cog_msg);

    arm::msg::TorqueEstimate torque_msg;
    torque_msg.header.stamp = stamp;
    torque_msg.header.frame_id = frame_id_;
    module_->fillTorqueEstimate(torque_msg);
    torque_pub_->publish(torque_msg);
}

}  // namespace DeltaArmRos
