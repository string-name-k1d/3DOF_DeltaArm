#ifndef ARM_CONNECTOR__PROTOCOL_HPP_
#define ARM_CONNECTOR__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "arm_connector/descriptor.hpp"

namespace DeltaArm::Connector
{

/**
 * @brief On-wire command byte (Structure.md "Commands").
 *
 * Discovery / Disconnect / Ping / ACK / Ctrl are the transport-level commands.
 * The handshake messages are listed separately because the spec sends them
 * "as ACK": they use the same frame layout but carry their own command byte so
 * the state machine stays readable. The wire values are stable.
 */
enum class Command : std::uint8_t
{
  kDiscovery = 0x01,      // arm -> actuator: discovery probe
  kDisconnect = 0x02,     // arm -> actuator: teardown request
  kPing = 0x03,           // keepalive / liveness check
  kAck = 0x04,            // generic acknowledgement
  kCtrl = 0x05,           // actuator-specific control payload
  kDevTypeId = 0x10,      // actuator -> arm: identity descriptor (handshake 1)
  kReqConnection = 0x11,  // arm -> actuator: request to mount (handshake 2)
  kReady = 0x12,          // both ways: ready to mount / ready to control
};

enum class AckResult : std::uint8_t
{
  kOk = 0,
  kRejected = 1,
  kBusy = 2,
  kUnknownCommand = 3,
  kTimeout = 4,
};

/**
 * @brief Classic-CAN payload: command + sequence + up to 6 data bytes.
 *
 * Classic CAN carries 8 bytes. CAN FD / multi-frame Ctrl payloads are a TODO
 * (see README.md); the handshake deliberately fits in one frame.
 */
constexpr std::size_t kMaxPayload = 6;
constexpr std::size_t kEncodedSize = 2 + kMaxPayload;

struct Frame
{
  Command command{Command::kDiscovery};
  std::uint8_t sequence{0};
  std::array<std::uint8_t, kMaxPayload> payload{};
  std::uint8_t payload_len{0};
};

/** Encoded frame plus its exact on-wire length (the CAN DLC). */
struct EncodedFrame
{
  std::array<std::uint8_t, kEncodedSize> data{};
  std::uint8_t size{0};
};

// Encode/decode the classic-CAN payload. encodeFrame() zero-pads to 8 bytes
// but reports the real length; decodeFrame() takes the received DLC.
EncodedFrame encodeFrame(const Frame & frame);
std::optional<Frame> decodeFrame(const std::uint8_t * data, std::size_t size);

// Frame builders. The ACK payload is [acked command, result]; the DevTypeId
// payload is [dev_type_id_lo, dev_type_id_hi, protocol_version].
Frame makeAck(std::uint8_t sequence, Command acked, AckResult result);
Frame makeDevTypeId(std::uint8_t sequence, DevTypeId id, std::uint8_t protocol_version);
Frame makeCtrl(std::uint8_t sequence, const std::uint8_t * payload, std::uint8_t len);
Frame makePing(std::uint8_t sequence);
bool parseDevTypeId(const Frame & frame, DevTypeId & id, std::uint8_t & protocol_version);

bool isKnownCommand(Command command) noexcept;
const char * toString(Command command) noexcept;
const char * toString(AckResult result) noexcept;

}  // namespace DeltaArm::Connector

#endif  // ARM_CONNECTOR__PROTOCOL_HPP_
