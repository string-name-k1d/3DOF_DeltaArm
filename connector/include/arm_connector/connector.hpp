#ifndef ARM_CONNECTOR__CONNECTOR_HPP_
#define ARM_CONNECTOR__CONNECTOR_HPP_

#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>

#include "arm_connector/actuator_control.hpp"
#include "arm_connector/descriptor.hpp"
#include "arm_connector/protocol.hpp"
#include "arm_connector/transport.hpp"

namespace DeltaArm::Connector
{

/** Arm-connector states (Structure.md "Arm-Connector States"). */
enum class State
{
  kDisconnected,
  kHandshake,
  kMounting,
  kUnmounting,
  kConnected,
};

const char * toString(State state) noexcept;

struct Config
{
  // CAN addressing. Discovery uses discovery_can_id (typically broadcast);
  // once the actuator has answered, all traffic uses arm_can_id / actuator_can_id.
  std::uint32_t arm_can_id{0x100};
  std::uint32_t actuator_can_id{0x101};
  std::uint32_t discovery_can_id{0x7FF};
  bool extended_ids{false};

  // Timing. Structure.md: ping every 500 ms; every wait has a timeout.
  std::chrono::milliseconds ping_period{500};
  std::chrono::milliseconds ack_timeout{100};
  std::chrono::milliseconds handshake_timeout{2000};
  std::chrono::milliseconds mounting_timeout{2000};
  std::chrono::milliseconds unmount_timeout{1000};

  // Retries before the link is declared lost.
  int max_retries{3};
};

/**
 * @brief Arm-side connector: the initiator and owner of the link state.
 *
 * The state machine follows arm/docs/Structure.md:
 *
 *   Disconnected --start()--> Handshake --DevTypeId--> (ReqConnection)
 *        ^                        |                         |
 *        |                        +-------- Ready <---------+
 *        |                                                  v
 *        |                                             Mounting --Ping/ACK--> Connected
 *        |                                                  |                    |
 *        +------------------ Unmounting <----- stop() ------+--------------------+
 *                                                                  keepalive ping
 *
 * This is a skeleton: the transitions, wire helpers and timeouts are in place;
 * actuator-specific Ctrl payloads live behind ActuatorControl.
 *
 * ROS integration is deliberately not in this ROS-free library. The intended
 * wrapper is arm/src/connector/connector_node.cpp:
 *   - rclcpp_lifecycle node: on_configure -> transport.open()
 *                             on_activate  -> connector.start()
 *                             on_deactivate-> connector.stop()
 *                             on_cleanup   -> transport.close()
 *   - feeds EndpointModule::setConnectorConnected(connector.connected()).
 */
class Connector
{
public:
  explicit Connector(Transport & transport, Config config = {});

  // Optional actuator-specific control protocol, resolved by DevTypeId.
  void setControlRegistry(const ActuatorControlRegistry * registry) { controls_ = registry; }

  /** Disconnected -> Handshake: send the discovery probe. */
  bool start();

  /** Request a graceful teardown (-> Unmounting -> Disconnected). */
  void stop();

  /** Drive timeouts, retries and the 500 ms keepalive. Call periodically. */
  void tick(std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());

  /** Non-blocking receive + dispatch. */
  void poll();

  /** Feed one received frame (used by poll() and by tests). */
  void handleFrame(const CanMessage & message);

  /** Send an actuator-specific Ctrl frame (requires Connected + registry). */
  bool sendControl(const std::vector<std::uint8_t> & args);

  State state() const noexcept { return state_; }
  const DeviceDescriptor & descriptor() const noexcept { return descriptor_; }
  bool connected() const noexcept { return state_ == State::kConnected; }

  // Observers.
  std::function<void(const DeviceDescriptor &)> on_connected;
  std::function<void()> on_disconnected;
  std::function<void(const Frame &)> on_ctrl_reply;
  std::function<void(State, const char *)> on_state_changed;

private:
  bool send(Command command, std::uint32_t can_id, const std::uint8_t * payload = nullptr,
            std::uint8_t len = 0);
  bool sendFrame(const Frame & frame, std::uint32_t can_id);
  void sendAck(std::uint8_t sequence, Command acked, AckResult result);
  std::uint32_t actuatorId() const;
  std::uint8_t nextSequence();
  void transition(State next, const char * reason);
  void failAndReset(const char * reason);

  Transport & transport_;
  Config config_;
  const ActuatorControlRegistry * controls_{nullptr};

  State state_{State::kDisconnected};
  DeviceDescriptor descriptor_;

  std::uint8_t sequence_{0};
  int retries_{0};
  bool waiting_ack_{false};

  std::chrono::steady_clock::time_point deadline_{};
  std::chrono::steady_clock::time_point last_ping_{};
};

}  // namespace DeltaArm::Connector

#endif  // ARM_CONNECTOR__CONNECTOR_HPP_
