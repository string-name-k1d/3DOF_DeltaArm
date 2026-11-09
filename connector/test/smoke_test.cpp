#include <cassert>
#include <chrono>
#include <iostream>

#include "arm_connector/connector.hpp"
#include "arm_connector/loopback_transport.hpp"

using namespace DeltaArm::Connector;

namespace
{
CanMessage message(std::uint32_t can_id, const Frame & frame)
{
  CanMessage out;
  out.can_id = can_id;
  out.frame = frame;
  return out;
}
}  // namespace

int main()
{
  // 1) Protocol round-trip.
  {
    const Frame ping = makePing(7);
    const auto encoded = encodeFrame(ping);
    const auto decoded = decodeFrame(encoded.data.data(), encoded.size);
    assert(decoded.has_value());
    assert(decoded->command == Command::kPing);
    assert(decoded->sequence == 7);
    assert(decoded->payload_len == 0);
  }

  {
    const Frame ack = makeAck(7, Command::kPing, AckResult::kOk);
    assert(ack.payload_len == 2);
    assert(ack.payload[0] == static_cast<std::uint8_t>(Command::kPing));
    assert(ack.payload[1] == static_cast<std::uint8_t>(AckResult::kOk));
  }

  {
    const Frame dev = makeDevTypeId(3, DevTypeId::kPogoActuatorGen0, 1);
    DevTypeId id = DevTypeId::kUnknown;
    std::uint8_t version = 0;
    assert(parseDevTypeId(dev, id, version));
    assert(id == DevTypeId::kPogoActuatorGen0);
    assert(version == 1);
  }

  // 2) Handshake over the loopback transport.
  LoopbackTransport transport;
  assert(transport.open("loopback", 0));

  Config config;
  config.actuator_can_id = 0x101;

  Connector connector(transport, config);
  bool connected = false;
  connector.on_connected = [&](const DeviceDescriptor & descriptor) {
      connected = descriptor.valid();
    };

  assert(connector.start());
  assert(connector.state() == State::kHandshake);

  // Handshake 1: actuator reports its DevTypeId.
  transport.inject(message(0x101, makeDevTypeId(0, DevTypeId::kPogoActuatorGen0, 1)));
  connector.poll();
  assert(connector.descriptor().dev_type_id == DevTypeId::kPogoActuatorGen0);

  // Handshake 2/3: actuator is ready to mount.
  Frame ready;
  ready.command = Command::kReady;
  ready.sequence = 1;
  transport.inject(message(0x101, ready));
  connector.poll();
  assert(connector.state() == State::kMounting);

  // Mounting: actuator ACKs the verification ping.
  const auto sent = transport.sent();
  assert(!sent.empty());
  assert(sent.back().frame.command == Command::kPing);
  transport.inject(message(0x101, makeAck(sent.back().frame.sequence, Command::kPing, AckResult::kOk)));
  connector.poll();
  assert(connector.connected());
  assert(connected);

  // 3) Keepalive: a tick after the ping period emits another Ping.
  const auto before = transport.sent().size();
  connector.tick(std::chrono::steady_clock::now() + std::chrono::milliseconds(600));
  assert(transport.sent().size() == before + 1);

  // 4) Graceful teardown.
  connector.stop();
  assert(connector.state() == State::kUnmounting);
  const auto sent_stop = transport.sent();
  assert(sent_stop.back().frame.command == Command::kDisconnect);
  transport.inject(
    message(0x101, makeAck(sent_stop.back().frame.sequence, Command::kDisconnect, AckResult::kOk)));
  connector.poll();
  assert(connector.state() == State::kDisconnected);

  std::cout << "arm_connector smoke test: ok\n";
  return 0;
}
