#ifndef ARM_CONNECTOR__TRANSPORT_HPP_
#define ARM_CONNECTOR__TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "arm_connector/protocol.hpp"

namespace DeltaArm::Connector
{

/** One CAN frame plus the arbitration metadata the transport needs. */
struct CanMessage
{
  std::uint32_t can_id{0};
  bool extended_id{false};
  Frame frame;
};

/**
 * @brief Byte-level transport for the connector protocol.
 *
 * The connector owns sequencing, timeouts, retries and the state machine; a
 * transport only moves frames. Implementations must be usable from a single
 * polling thread (the intended ROS integration calls poll() from a timer).
 */
class Transport
{
public:
  virtual ~Transport() = default;

  virtual bool open(const std::string & interface_name, std::uint32_t bitrate) = 0;
  virtual void close() = 0;
  virtual bool isOpen() const = 0;

  virtual bool send(const CanMessage & message, std::chrono::milliseconds timeout) = 0;
  virtual std::optional<CanMessage> receive(std::chrono::milliseconds timeout) = 0;

  virtual std::string lastError() const = 0;
};

}  // namespace DeltaArm::Connector

#endif  // ARM_CONNECTOR__TRANSPORT_HPP_
