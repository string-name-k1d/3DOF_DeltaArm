#include "arm_connector/loopback_transport.hpp"

#include <chrono>
#include <thread>
#include <utility>

namespace DeltaArm::Connector
{

bool LoopbackTransport::send(const CanMessage & message, std::chrono::milliseconds)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!open_) {
    return false;
  }
  outbound_.push_back(message);
  return true;
}

std::optional<CanMessage> LoopbackTransport::receive(std::chrono::milliseconds timeout)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!inbound_.empty()) {
      CanMessage message = std::move(inbound_.front());
      inbound_.pop_front();
      return message;
    }
  }

  // Mimic a blocking socket read for callers that pass a real timeout. The
  // ROS wrapper polls with 0 ms, so this only matters for tests/examples.
  if (timeout.count() > 0) {
    std::this_thread::sleep_for(timeout);
  }
  return std::nullopt;
}

void LoopbackTransport::inject(const CanMessage & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  inbound_.push_back(message);
}

std::deque<CanMessage> LoopbackTransport::sent() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return outbound_;
}

}  // namespace DeltaArm::Connector
