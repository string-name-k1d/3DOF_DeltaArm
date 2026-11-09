#ifndef ARM_CONNECTOR__DESCRIPTOR_HPP_
#define ARM_CONNECTOR__DESCRIPTOR_HPP_

#include <cstdint>
#include <cstdio>
#include <string>

namespace DeltaArm::Connector
{

/**
 * @brief Actuator family identifier (DevTypeId).
 *
 * The value selects the actuator-specific Ctrl protocol (see
 * actuator_control.hpp). Values are part of the wire format: append new
 * families, never renumber existing ones.
 */
enum class DevTypeId : std::uint16_t
{
  kUnknown = 0x0000,
  kPogoActuatorGen0 = 0x0001,
};

inline const char * toString(DevTypeId id) noexcept
{
  switch (id) {
    case DevTypeId::kUnknown:
      return "unknown";
    case DevTypeId::kPogoActuatorGen0:
      return "pogo_actuator_gen0";
  }
  return "unknown";
}

/**
 * @brief Identity descriptor of one physical actuator.
 *
 * The arm-side connector caches this after the handshake and uses dev_type_id
 * to pick the actuator-specific control protocol. The remaining fields are
 * informational (logs / diagnostics / future multi-actuator routing).
 *
 * Wire note: a classic CAN frame carries 8 bytes, so the skeleton handshake
 * moves the DevTypeId and protocol version only. vendor/product/serial are
 * populated from configuration today; a multi-frame descriptor exchange is a
 * documented TODO in README.md.
 */
struct DeviceDescriptor
{
  DevTypeId dev_type_id{DevTypeId::kUnknown};
  std::uint16_t vendor_id{0};
  std::uint16_t product_id{0};
  std::uint32_t serial{0};
  std::uint8_t protocol_version{1};

  // CAN arbitration id this actuator answers on (0 = unknown / discovery).
  std::uint32_t can_id{0};
  bool extended_id{false};

  bool valid() const noexcept { return dev_type_id != DevTypeId::kUnknown; }

  std::string toString() const
  {
    char buffer[96];
    std::snprintf(
      buffer, sizeof(buffer), "dev=%s(0x%04X) vendor=0x%04X product=0x%04X serial=%u proto=%u",
      DeltaArm::Connector::toString(dev_type_id), static_cast<unsigned>(dev_type_id), vendor_id,
      product_id, serial, protocol_version);
    return std::string(buffer);
  }
};

}  // namespace DeltaArm::Connector

#endif  // ARM_CONNECTOR__DESCRIPTOR_HPP_
