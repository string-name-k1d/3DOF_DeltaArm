#ifndef ARM_CONNECTOR__ACTUATOR_CONTROL_HPP_
#define ARM_CONNECTOR__ACTUATOR_CONTROL_HPP_

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "arm_connector/descriptor.hpp"
#include "arm_connector/protocol.hpp"

namespace DeltaArm::Connector
{

/**
 * @brief Actuator-specific control protocol, selected by DevTypeId.
 *
 * Structure.md: "Actuator Control Protocol should be custom to actuator,
 * identified by DevTypeId." Each actuator family implements this interface to
 * build Ctrl frames and interpret replies; the Connector only guarantees the
 * link (discovery / ping / ACK / timeouts).
 */
class ActuatorControl
{
public:
  virtual ~ActuatorControl() = default;

  virtual DevTypeId devTypeId() const = 0;

  /** Build a Ctrl frame from actuator-specific arguments. */
  virtual Frame makeCtrl(std::uint8_t sequence, const std::vector<std::uint8_t> & args) const = 0;

  /** Handle a Ctrl reply from the actuator. */
  virtual void onCtrlReply(const Frame & reply) = 0;
};

/**
 * @brief DevTypeId -> ActuatorControl lookup.
 *
 * Register one implementation per actuator family during bring-up; the
 * Connector resolves the control protocol after the handshake reports the
 * DevTypeId.
 */
class ActuatorControlRegistry
{
public:
  void add(std::shared_ptr<ActuatorControl> control)
  {
    if (control) {
      controls_[static_cast<std::uint16_t>(control->devTypeId())] = std::move(control);
    }
  }

  std::shared_ptr<ActuatorControl> find(DevTypeId id) const
  {
    const auto it = controls_.find(static_cast<std::uint16_t>(id));
    return it == controls_.end() ? nullptr : it->second;
  }

private:
  std::unordered_map<std::uint16_t, std::shared_ptr<ActuatorControl>> controls_;
};

}  // namespace DeltaArm::Connector

#endif  // ARM_CONNECTOR__ACTUATOR_CONTROL_HPP_
