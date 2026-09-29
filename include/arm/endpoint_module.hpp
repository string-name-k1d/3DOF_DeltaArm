#ifndef arm__ENDPOINT_MODULE_HPP_
#define arm__ENDPOINT_MODULE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

#include "arm/msg/cog_estimate.hpp"
#include "arm/msg/endpoint_state.hpp"
#include "arm/msg/torque_estimate.hpp"

namespace DeltaArmRos
{

/**
 * @brief Interface for a swappable terminal end-effector module.
 *
 * A module models whatever sits at the end of the arm (tool, gripper, sensor,
 * payload) and contributes the physical data the control stack needs:
 *   - connection state (attached / detached),
 *   - payload mass and centre of gravity,
 *   - centre-of-gravity and hold-torque estimates.
 *
 * Modules are selected by the arm_endpoint node via the `module_type`
 * parameter and created through makeEndpointModule(); adding a new real module
 * means adding a new implementation of this interface plus a factory branch —
 * no changes to the node or the controller.
 */
class EndpointModule
{
public:
  virtual ~EndpointModule() = default;

  // Human-readable identifier, e.g. "null" or "gripper".
  virtual std::string type() const = 0;

  // True for real (non-stub) hardware; false for the null placeholder.
  virtual bool isActive() const = 0;

  // Dummy connector flag for now. Real modules will read a limit switch here.
  virtual bool connectorConnected() const = 0;

  // Attach / detach the payload (updates connector state).
  virtual void setConnectorConnected(bool connected) = 0;

  // Payload mass in kg (0 for the null module).
  virtual double payloadMassKg() const = 0;

  // Payload CoG in the endpoint frame, meters.
  virtual geometry_msgs::msg::Point payloadCoG() const = 0;

  // Combined CoG estimate (arm + payload) in the base frame.
  virtual geometry_msgs::msg::Point estimateCogBase() const = 0;

  // Hold-torque estimate in Nm at the endpoint (r x F for the stub).
  virtual geometry_msgs::msg::Vector3 estimateTorque() const = 0;

  // Fill a state message with this module's current values.
  virtual void fillEndpointState(arm::msg::EndpointState & msg) const = 0;

  // Fill a CoG estimate message.
  virtual void fillCogEstimate(arm::msg::CogEstimate & msg) const = 0;

  // Fill a torque estimate message.
  virtual void fillTorqueEstimate(arm::msg::TorqueEstimate & msg) const = 0;

  // Factory: create a module by type name. Unknown types fall back to "null".
  // The node reference is non-const because modules declare their parameters.
  static std::shared_ptr<EndpointModule> makeEndpointModule(const std::string & type, rclcpp::Node & node);
};

/**
 * @brief Null (placeholder) endpoint module: no real hardware, but exposes the
 * full interface so the rest of the stack can be built and tested now.
 *
 * The dummy `connector_connected_` flag can be toggled over the
 * `arm/endpoint/attach` (std_srvs/SetBool) service; when set, a synthetic
 * payload mass/CoG is reported so the CoG and torque estimators produce
 * non-trivial values.
 */
class NullEndpointModule : public EndpointModule
{
public:
  explicit NullEndpointModule(rclcpp::Node & node);

  std::string type() const override { return "null"; }
  bool isActive() const override { return false; }
  bool connectorConnected() const override { return connector_connected_; }
  void setConnectorConnected(bool connected) override;

  double payloadMassKg() const override;
  geometry_msgs::msg::Point payloadCoG() const override;
  geometry_msgs::msg::Point estimateCogBase() const override;
  geometry_msgs::msg::Vector3 estimateTorque() const override;

  void fillEndpointState(arm::msg::EndpointState & msg) const override;
  void fillCogEstimate(arm::msg::CogEstimate & msg) const override;
  void fillTorqueEstimate(arm::msg::TorqueEstimate & msg) const override;

  // Combined CoG of the arm alone (used when no payload is attached).
  const geometry_msgs::msg::Point & armCoG() const noexcept { return arm_cog_; }
  double armMassKg() const noexcept { return arm_mass_kg_; }

private:
  // Synthetic payload parameters (used only when the dummy connector is set).
  double payload_mass_kg_;
  geometry_msgs::msg::Point payload_cog_;
  geometry_msgs::msg::Point arm_cog_;
  double arm_mass_kg_;
  bool connector_connected_;
};

}  // namespace DeltaArmRos

#endif  // arm__ENDPOINT_MODULE_HPP_
