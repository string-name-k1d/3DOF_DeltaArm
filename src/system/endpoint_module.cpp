#include "arm/endpoint_module.hpp"

#include <cmath>
#include <utility>

namespace DeltaArmRos {

namespace {
constexpr double kGravity = 9.81;
}  // namespace

// ---------------------------------------------------------------------------
// NullEndpointModule
// ---------------------------------------------------------------------------
NullEndpointModule::NullEndpointModule(rclcpp::Node & node)
    : payload_mass_kg_(node.declare_parameter<double>("payload_mass_kg", 0.25)),
      payload_cog_(),
      arm_cog_(),
      arm_mass_kg_(node.declare_parameter<double>("arm_mass_kg", 1.20)),
      connector_connected_(node.declare_parameter<bool>("connector_connected", false)) {
    // Arm's own CoG, roughly at the platform height in the base frame.
    arm_cog_.x = node.declare_parameter<double>("arm_cog_x", 0.0);
    arm_cog_.y = node.declare_parameter<double>("arm_cog_y", 0.0);
    arm_cog_.z = node.declare_parameter<double>("arm_cog_z", -0.12);

    // Synthetic payload CoG in the endpoint frame (a tool hanging off the tip).
    payload_cog_.x = node.declare_parameter<double>("payload_cog_x", 0.0);
    payload_cog_.y = node.declare_parameter<double>("payload_cog_y", 0.0);
    payload_cog_.z = node.declare_parameter<double>("payload_cog_z", 0.04);
}

void NullEndpointModule::setConnectorConnected(bool connected) { connector_connected_ = connected; }

double NullEndpointModule::payloadMassKg() const { return connector_connected_ ? payload_mass_kg_ : 0.0; }

geometry_msgs::msg::Point NullEndpointModule::payloadCoG() const { return payload_cog_; }

geometry_msgs::msg::Point NullEndpointModule::estimateCogBase() const {
    // Mass-weighted blend of the arm CoG and the (attached) payload CoG. The
    // payload CoG is treated as coincident with the endpoint for this skeleton.
    const double m_arm = arm_mass_kg_;
    const double m_pay = payloadMassKg();
    if (m_arm <= 0.0) {
        return payload_cog_;
    }
    const double total = m_arm + m_pay;
    geometry_msgs::msg::Point cog;
    // Endpoint-frame payload CoG is mapped to base by the caller via the
    // endpoint pose; for the null module we use the arm CoG as the reference
    // so the result is a valid, stable point.
    cog.x = (arm_cog_.x * m_arm + payload_cog_.x * m_pay) / total;
    cog.y = (arm_cog_.y * m_arm + payload_cog_.y * m_pay) / total;
    cog.z = (arm_cog_.z * m_arm + (arm_cog_.z + payload_cog_.z) * m_pay) / total;
    return cog;
}

geometry_msgs::msg::Vector3 NullEndpointModule::estimateTorque() const {
    // Hold-torque estimate: tau = r x F, with F the weight acting at the CoG
    // and r the lever arm from the joint axis to that CoG. We take the joint
    // axis as the horizontal x-axis offset, so the torque is about y (x) and
    // x (-z). This is the standard gravity-hold cross product.
    const geometry_msgs::msg::Point cog = estimateCogBase();
    const double m_total = arm_mass_kg_ + payloadMassKg();

    // F = m * g  (downward, -z)
    const double fz = -m_total * kGravity;

    // r from the joint axis (origin) to the CoG.
    const double rx = cog.x;
    const double ry = cog.y;
    const double rz = cog.z;

    geometry_msgs::msg::Vector3 tau;
    // r x F, with F = (0, 0, fz):
    tau.x = ry * fz - rz * 0.0;
    tau.y = rz * 0.0 - rx * fz;
    tau.z = rx * 0.0 - ry * 0.0;
    return tau;
}

void NullEndpointModule::fillEndpointState(arm::msg::EndpointState & msg) const {
    msg.module_type = type();
    msg.connector_connected = connector_connected_;
    msg.active = isActive();
    msg.payload_mass_kg = static_cast<float>(payloadMassKg());
    msg.payload_cog = payloadCoG();
    msg.status = connector_connected_ ? "null module, dummy payload attached" : "null module, no payload";
}

void NullEndpointModule::fillCogEstimate(arm::msg::CogEstimate & msg) const {
    msg.cog_base = estimateCogBase();
    msg.cog_endpoint = payloadCoG();
    msg.total_mass_kg = static_cast<float>(arm_mass_kg_ + payloadMassKg());
    msg.confidence = connector_connected_ ? 1.0f : 0.5f;
}

void NullEndpointModule::fillTorqueEstimate(arm::msg::TorqueEstimate & msg) const {
    const geometry_msgs::msg::Point cog = estimateCogBase();
    const double m_total = arm_mass_kg_ + payloadMassKg();

    msg.torque = estimateTorque();
    msg.lever_arm.x = cog.x;
    msg.lever_arm.y = cog.y;
    msg.lever_arm.z = cog.z;
    msg.force.x = 0.0;
    msg.force.y = 0.0;
    msg.force.z = -m_total * kGravity;

    // Distribute the end-effector hold torque evenly across the 3 legs.
    for (int i = 0; i < 3; ++i) {
        msg.joint_torque[i] = static_cast<float>(msg.torque.y / 3.0);
    }
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
std::shared_ptr<EndpointModule> EndpointModule::makeEndpointModule(const std::string & type, rclcpp::Node & node) {
    if (type == "null" || type.empty()) {
        return std::make_shared<NullEndpointModule>(node);
    }
    // No real modules yet: unknown types degrade to the null placeholder.
    RCLCPP_WARN(node.get_logger(), "endpoint module '%s' not implemented - falling back to 'null'", type.c_str());
    return std::make_shared<NullEndpointModule>(node);
}

}  // namespace DeltaArmRos
