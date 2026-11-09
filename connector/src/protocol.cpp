#include "arm_connector/protocol.hpp"

#include <algorithm>

namespace DeltaArm::Connector
{

EncodedFrame encodeFrame(const Frame & frame)
{
  EncodedFrame out;
  out.size = static_cast<std::uint8_t>(2U + std::min<std::size_t>(frame.payload_len, kMaxPayload));
  out.data[0] = static_cast<std::uint8_t>(frame.command);
  out.data[1] = frame.sequence;
  std::copy_n(frame.payload.begin(), frame.payload_len, out.data.begin() + 2);
  return out;
}

std::optional<Frame> decodeFrame(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr || size < 2 || size > kEncodedSize) {
    return std::nullopt;
  }

  Frame frame;
  frame.command = static_cast<Command>(data[0]);
  frame.sequence = data[1];
  frame.payload_len = static_cast<std::uint8_t>(size - 2);
  std::copy_n(data + 2, frame.payload_len, frame.payload.begin());
  return frame;
}

Frame makeAck(std::uint8_t sequence, Command acked, AckResult result)
{
  Frame frame;
  frame.command = Command::kAck;
  frame.sequence = sequence;
  frame.payload[0] = static_cast<std::uint8_t>(acked);
  frame.payload[1] = static_cast<std::uint8_t>(result);
  frame.payload_len = 2;
  return frame;
}

Frame makeDevTypeId(std::uint8_t sequence, DevTypeId id, std::uint8_t protocol_version)
{
  Frame frame;
  frame.command = Command::kDevTypeId;
  frame.sequence = sequence;
  const auto raw = static_cast<std::uint16_t>(id);
  frame.payload[0] = static_cast<std::uint8_t>(raw & 0xFFU);
  frame.payload[1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
  frame.payload[2] = protocol_version;
  frame.payload_len = 3;
  return frame;
}

Frame makeCtrl(std::uint8_t sequence, const std::uint8_t * payload, std::uint8_t len)
{
  Frame frame;
  frame.command = Command::kCtrl;
  frame.sequence = sequence;
  frame.payload_len = std::min<std::uint8_t>(len, kMaxPayload);
  if (payload != nullptr) {
    std::copy_n(payload, frame.payload_len, frame.payload.begin());
  }
  return frame;
}

Frame makePing(std::uint8_t sequence)
{
  Frame frame;
  frame.command = Command::kPing;
  frame.sequence = sequence;
  frame.payload_len = 0;
  return frame;
}

bool parseDevTypeId(const Frame & frame, DevTypeId & id, std::uint8_t & protocol_version)
{
  if (frame.command != Command::kDevTypeId || frame.payload_len < 3) {
    return false;
  }
  const auto raw = static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(frame.payload[0]) |
    (static_cast<std::uint16_t>(frame.payload[1]) << 8U));
  id = static_cast<DevTypeId>(raw);
  protocol_version = frame.payload[2];
  return true;
}

bool isKnownCommand(Command command) noexcept
{
  switch (command) {
    case Command::kDiscovery:
    case Command::kDisconnect:
    case Command::kPing:
    case Command::kAck:
    case Command::kCtrl:
    case Command::kDevTypeId:
    case Command::kReqConnection:
    case Command::kReady:
      return true;
  }
  return false;
}

const char * toString(Command command) noexcept
{
  switch (command) {
    case Command::kDiscovery:
      return "discovery";
    case Command::kDisconnect:
      return "disconnect";
    case Command::kPing:
      return "ping";
    case Command::kAck:
      return "ack";
    case Command::kCtrl:
      return "ctrl";
    case Command::kDevTypeId:
      return "dev_type_id";
    case Command::kReqConnection:
      return "req_connection";
    case Command::kReady:
      return "ready";
  }
  return "unknown";
}

const char * toString(AckResult result) noexcept
{
  switch (result) {
    case AckResult::kOk:
      return "ok";
    case AckResult::kRejected:
      return "rejected";
    case AckResult::kBusy:
      return "busy";
    case AckResult::kUnknownCommand:
      return "unknown_command";
    case AckResult::kTimeout:
      return "timeout";
  }
  return "unknown";
}

}  // namespace DeltaArm::Connector
