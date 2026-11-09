#include "arm_connector/connector.hpp"

#include <cstddef>

namespace DeltaArm::Connector
{

namespace
{
bool hasAckPayload(const Frame & frame)
{
  return frame.payload_len >= 2;
}
}  // namespace

const char * toString(State state) noexcept
{
  switch (state) {
    case State::kDisconnected:
      return "disconnected";
    case State::kHandshake:
      return "handshake";
    case State::kMounting:
      return "mounting";
    case State::kUnmounting:
      return "unmounting";
    case State::kConnected:
      return "connected";
  }
  return "unknown";
}

Connector::Connector(Transport & transport, Config config)
: transport_(transport), config_(config)
{
}

std::uint8_t Connector::nextSequence() { return sequence_++; }

std::uint32_t Connector::actuatorId() const
{
  return descriptor_.can_id != 0U ? descriptor_.can_id : config_.actuator_can_id;
}

bool Connector::sendFrame(const Frame & frame, std::uint32_t can_id)
{
  CanMessage message;
  message.can_id = can_id;
  message.extended_id = config_.extended_ids;
  message.frame = frame;
  return transport_.send(message, config_.ack_timeout);
}

bool Connector::send(Command command, std::uint32_t can_id, const std::uint8_t * payload,
                     std::uint8_t len)
{
  Frame frame;
  frame.command = command;
  frame.sequence = nextSequence();
  frame.payload_len = len > kMaxPayload ? kMaxPayload : len;
  for (std::size_t i = 0; i < frame.payload_len; ++i) {
    frame.payload[i] = payload[i];
  }
  return sendFrame(frame, can_id);
}

void Connector::sendAck(std::uint8_t sequence, Command acked, AckResult result)
{
  sendFrame(makeAck(sequence, acked, result), actuatorId());
}

void Connector::transition(State next, const char * reason)
{
  if (state_ == next) {
    return;
  }
  state_ = next;
  if (on_state_changed) {
    on_state_changed(state_, reason);
  }
}

void Connector::failAndReset(const char * reason)
{
  waiting_ack_ = false;
  retries_ = 0;
  transition(State::kDisconnected, reason);
  if (on_disconnected) {
    on_disconnected();
  }
}

bool Connector::start()
{
  if (state_ != State::kDisconnected) {
    return false;
  }

  descriptor_ = {};
  retries_ = 0;
  waiting_ack_ = false;

  if (!send(Command::kDiscovery, config_.discovery_can_id)) {
    return false;
  }

  deadline_ = std::chrono::steady_clock::now() + config_.handshake_timeout;
  transition(State::kHandshake, "discovery probe sent");
  return true;
}

void Connector::stop()
{
  if (state_ == State::kDisconnected || state_ == State::kUnmounting) {
    return;
  }

  send(Command::kDisconnect, actuatorId());
  deadline_ = std::chrono::steady_clock::now() + config_.unmount_timeout;
  transition(State::kUnmounting, "disconnect requested");
}

void Connector::poll()
{
  while (auto message = transport_.receive(std::chrono::milliseconds(0))) {
    handleFrame(*message);
  }
}

void Connector::handleFrame(const CanMessage & message)
{
  const Frame & frame = message.frame;

  // Structure.md rule 3: every message is responded to with an ACK.
  if (frame.command != Command::kAck) {
    sendAck(
      frame.sequence, frame.command,
      isKnownCommand(frame.command) ? AckResult::kOk : AckResult::kUnknownCommand);
  }

  switch (frame.command) {
    case Command::kDevTypeId: {
      if (state_ != State::kHandshake) {
        break;
      }
      DevTypeId id = DevTypeId::kUnknown;
      std::uint8_t version = 0;
      if (!parseDevTypeId(frame, id, version)) {
        break;
      }

      descriptor_.dev_type_id = id;
      descriptor_.protocol_version = version;
      descriptor_.can_id = message.can_id;
      descriptor_.extended_id = message.extended_id;

      // Handshake 2: request the connection.
      retries_ = 0;
      send(Command::kReqConnection, actuatorId());
      deadline_ = std::chrono::steady_clock::now() + config_.handshake_timeout;
      break;
    }

    case Command::kReady: {
      if (state_ != State::kHandshake) {
        break;
      }

      // Handshake 3: the actuator is ready. Enter Mounting, send the arm-side
      // Ready, then verify with a Ping.
      retries_ = 0;
      transition(State::kMounting, "handshake complete");
      send(Command::kReady, actuatorId());
      if (send(Command::kPing, actuatorId())) {
        waiting_ack_ = true;
        deadline_ = std::chrono::steady_clock::now() + config_.mounting_timeout;
      }
      break;
    }

    case Command::kAck: {
      if (!hasAckPayload(frame)) {
        break;
      }
      const auto acked = static_cast<Command>(frame.payload[0]);
      const auto result = static_cast<AckResult>(frame.payload[1]);

      if (state_ == State::kMounting && acked == Command::kPing &&
        result == AckResult::kOk)
      {
        waiting_ack_ = false;
        last_ping_ = std::chrono::steady_clock::now();
        retries_ = 0;
        transition(State::kConnected, "mount verified");
        if (on_connected) {
          on_connected(descriptor_);
        }
      } else if (state_ == State::kUnmounting && acked == Command::kDisconnect) {
        transition(State::kDisconnected, "actuator acknowledged disconnect");
        if (on_disconnected) {
          on_disconnected();
        }
      } else if (state_ == State::kConnected && acked == Command::kPing) {
        waiting_ack_ = false;
        retries_ = 0;
      }
      break;
    }

    case Command::kCtrl: {
      if (state_ != State::kConnected) {
        break;
      }
      if (on_ctrl_reply) {
        on_ctrl_reply(frame);
      }
      if (controls_) {
        if (auto control = controls_->find(descriptor_.dev_type_id)) {
          control->onCtrlReply(frame);
        }
      }
      break;
    }

    case Command::kDisconnect: {
      // Actuator-initiated teardown.
      if (state_ != State::kDisconnected) {
        failAndReset("actuator requested disconnect");
      }
      break;
    }

    case Command::kDiscovery:
    case Command::kPing:
    case Command::kReqConnection:
    default:
      break;
  }
}

void Connector::tick(std::chrono::steady_clock::time_point now)
{
  switch (state_) {
    case State::kDisconnected:
      return;

    case State::kHandshake:
      if (now >= deadline_) {
        if (retries_ < config_.max_retries) {
          ++retries_;
          send(Command::kDiscovery, config_.discovery_can_id);
          deadline_ = now + config_.handshake_timeout;
        } else {
          failAndReset("handshake timeout");
        }
      }
      return;

    case State::kMounting:
      if (now >= deadline_) {
        if (retries_ < config_.max_retries) {
          ++retries_;
          send(Command::kPing, actuatorId());
          deadline_ = now + config_.mounting_timeout;
        } else {
          failAndReset("mounting timeout");
        }
      }
      return;

    case State::kUnmounting:
      if (now >= deadline_) {
        transition(State::kDisconnected, "unmount timeout");
        if (on_disconnected) {
          on_disconnected();
        }
      }
      return;

    case State::kConnected: {
      if (now - last_ping_ >= config_.ping_period) {
        last_ping_ = now;
        if (!send(Command::kPing, actuatorId())) {
          failAndReset("ping send failed");
          return;
        }
        waiting_ack_ = true;
        deadline_ = now + config_.ack_timeout;
      }

      if (waiting_ack_ && now >= deadline_) {
        waiting_ack_ = false;
        if (++retries_ > config_.max_retries) {
          failAndReset("keepalive lost");
        }
      }
      return;
    }
  }
}

bool Connector::sendControl(const std::vector<std::uint8_t> & args)
{
  if (state_ != State::kConnected || controls_ == nullptr) {
    return false;
  }

  auto control = controls_->find(descriptor_.dev_type_id);
  if (!control) {
    return false;
  }

  return sendFrame(control->makeCtrl(nextSequence(), args), actuatorId());
}

}  // namespace DeltaArm::Connector
