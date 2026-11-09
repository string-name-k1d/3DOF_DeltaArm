#ifndef ARM_CONNECTOR__LOOPBACK_TRANSPORT_HPP_
#define ARM_CONNECTOR__LOOPBACK_TRANSPORT_HPP_

#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

#include "arm_connector/transport.hpp"

namespace DeltaArm::Connector
{

/**
 * @brief In-process transport for tests and bench work without CAN hardware.
 *
 * Frames sent by the connector are appended to sent(); a test (or a mock
 * actuator) pushes replies with inject(). This is what the smoke test uses to
 * exercise the handshake without a physical bus.
 */
class LoopbackTransport final : public Transport
{
public:
  bool open(const std::string &, std::uint32_t) override
  {
    open_ = true;
    return true;
  }
  void close() override { open_ = false; }
  bool isOpen() const override { return open_; }

  bool send(const CanMessage & message, std::chrono::milliseconds timeout) override;
  std::optional<CanMessage> receive(std::chrono::milliseconds timeout) override;

  void inject(const CanMessage & message);
  std::deque<CanMessage> sent() const;

  std::string lastError() const override { return {}; }

private:
  mutable std::mutex mutex_;
  std::deque<CanMessage> inbound_;
  std::deque<CanMessage> outbound_;
  bool open_{false};
};

}  // namespace DeltaArm::Connector

#endif  // ARM_CONNECTOR__LOOPBACK_TRANSPORT_HPP_
