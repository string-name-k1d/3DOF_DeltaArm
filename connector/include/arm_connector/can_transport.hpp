#ifndef ARM_CONNECTOR__CAN_TRANSPORT_HPP_
#define ARM_CONNECTOR__CAN_TRANSPORT_HPP_

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "arm_connector/transport.hpp"

namespace DeltaArm::Connector
{

/**
 * @brief Linux SocketCAN transport for the pogo-pin CAN link.
 *
 * Skeleton: opens a SocketCAN interface (e.g. "can0") and reads/writes classic
 * CAN frames. The bus bitrate is normally configured outside the process:
 *
 *   ip link set can0 type can bitrate 1000000
 *   ip link set can0 up
 *
 * The bitrate argument is recorded for diagnostics only; it does not
 * reconfigure the kernel interface.
 */
class CanTransport final : public Transport
{
public:
  CanTransport() = default;
  ~CanTransport() override;

  CanTransport(const CanTransport &) = delete;
  CanTransport & operator=(const CanTransport &) = delete;

  bool open(const std::string & interface_name, std::uint32_t bitrate) override;
  void close() override;
  bool isOpen() const override { return socket_ >= 0; }

  bool send(const CanMessage & message, std::chrono::milliseconds timeout) override;
  std::optional<CanMessage> receive(std::chrono::milliseconds timeout) override;

  std::string lastError() const override { return error_; }

private:
  int socket_{-1};
  std::string interface_;
  std::uint32_t bitrate_{0};
  std::string error_;
};

}  // namespace DeltaArm::Connector

#endif  // ARM_CONNECTOR__CAN_TRANSPORT_HPP_
