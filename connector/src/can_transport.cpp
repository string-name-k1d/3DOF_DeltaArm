#include "arm_connector/can_transport.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>

namespace DeltaArm::Connector
{

CanTransport::~CanTransport() { close(); }

bool CanTransport::open(const std::string & interface_name, std::uint32_t bitrate)
{
  close();
  error_.clear();
  interface_ = interface_name;
  bitrate_ = bitrate;

  socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_ < 0) {
    error_ = "socket(PF_CAN) failed";
    return false;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
    error_ = "SIOCGIFINDEX failed for " + interface_name;
    close();
    return false;
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (::bind(socket_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    error_ = "bind(AF_CAN) failed for " + interface_name;
    close();
    return false;
  }

  return true;
}

void CanTransport::close()
{
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
}

bool CanTransport::send(const CanMessage & message, std::chrono::milliseconds timeout)
{
  if (socket_ < 0) {
    error_ = "transport not open";
    return false;
  }

  const auto encoded = encodeFrame(message.frame);

  struct can_frame raw {};
  raw.can_id = message.can_id & (message.extended_id ? CAN_EFF_MASK : CAN_SFF_MASK);
  if (message.extended_id) {
    raw.can_id |= CAN_EFF_FLAG;
  }
  raw.can_dlc = encoded.size;
  std::memcpy(raw.data, encoded.data.data(), encoded.size);

  struct pollfd pfd {};
  pfd.fd = socket_;
  pfd.events = POLLOUT;
  const int ready = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (ready <= 0) {
    error_ = ready == 0 ? "CAN send timeout" : "CAN poll failed";
    return false;
  }

  const ssize_t written = ::write(socket_, &raw, sizeof(raw));
  if (written != static_cast<ssize_t>(sizeof(raw))) {
    error_ = "CAN write failed";
    return false;
  }
  return true;
}

std::optional<CanMessage> CanTransport::receive(std::chrono::milliseconds timeout)
{
  if (socket_ < 0) {
    error_ = "transport not open";
    return std::nullopt;
  }

  struct pollfd pfd {};
  pfd.fd = socket_;
  pfd.events = POLLIN;
  const int ready = ::poll(&pfd, 1, static_cast<int>(timeout.count()));
  if (ready <= 0) {
    return std::nullopt;
  }

  struct can_frame raw {};
  const ssize_t bytes = ::read(socket_, &raw, sizeof(raw));
  if (bytes != static_cast<ssize_t>(sizeof(raw))) {
    return std::nullopt;
  }

  auto frame = decodeFrame(raw.data, raw.can_dlc);
  if (!frame) {
    return std::nullopt;
  }

  CanMessage message;
  message.extended_id = (raw.can_id & CAN_EFF_FLAG) != 0;
  message.can_id = raw.can_id & (message.extended_id ? CAN_EFF_MASK : CAN_SFF_MASK);
  message.frame = *frame;
  return message;
}

}  // namespace DeltaArm::Connector
