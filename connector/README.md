# Arm-Actuator Connector (skeleton)

Arm-side connector for the pogo-pin **CAN** link to an external actuator at the
endpoint. This directory is a **standalone, ROS-free C++17 library** so the
protocol can be built and tested without ROS or the arm stack.

Spec: [`../docs/Structure.md`](../docs/Structure.md).

```
connector/
├── include/arm_connector/
│   ├── descriptor.hpp        # DevTypeId + DeviceDescriptor (id descriptor)
│   ├── protocol.hpp          # Command, Frame, encode/decode, handshake builders
│   ├── transport.hpp         # Transport interface + CanMessage
│   ├── can_transport.hpp     # Linux SocketCAN transport (skeleton)
│   ├── loopback_transport.hpp# in-process transport for tests/bench work
│   ├── actuator_control.hpp  # DevTypeId-keyed Ctrl protocol + registry
│   └── connector.hpp         # states, timeouts, Config, Connector
├── src/                      # implementations
└── test/smoke_test.cpp       # protocol + handshake + teardown smoke test
```

## Layers (Structure.md)

| Layer | Where |
| --- | --- |
| Descriptor | `descriptor.hpp` (`DevTypeId`, `DeviceDescriptor`) |
| Discover + Transport | `protocol.hpp`, `transport.hpp`, `can_transport.hpp` |
| Actuator Control | `actuator_control.hpp` (custom protocol per `DevTypeId`) |
| Connector state machine | `connector.hpp` / `src/connector.cpp` |
| ROS lifecycle | **not in this library** - see "ROS integration" below |

## Wire format (classic CAN, 8 bytes)

```
byte 0        byte 1      bytes 2..7
command       sequence    payload (0..6 bytes)
```

Commands: `Discovery`, `Disconnect`, `Ping`, `ACK`, `Ctrl`, plus the handshake
messages `DevTypeId`, `ReqConnection`, `Ready` (sent "as ACK" per the spec).
`ACK` payload is `[acked command, result]`; `DevTypeId` payload is
`[dev_type_id_lo, dev_type_id_hi, protocol_version]`.

Every non-ACK message is answered with an ACK, and every wait has a timeout
(`Config::ack_timeout`, `handshake_timeout`, `mounting_timeout`,
`unmount_timeout`). Pings run every `Config::ping_period` (default 500 ms).

## State machine

```
Disconnected --start()--> Handshake --DevTypeId--> (ReqConnection)
     ^                        |                         |
     |                        +-------- Ready <---------+
     |                                                  v
     |                                             Mounting --Ping/ACK--> Connected
     |                                                  |                    |
     +------------------ Unmounting <----- stop() ------+--------------------+
                                                              keepalive ping
```

The arm side always initiates and owns the link (discovery, handshake,
keepalive, disconnect handling).

## Build / test standalone

```bash
cmake -S arm/connector -B /tmp/arm_connector
cmake --build /tmp/arm_connector
ctest --test-dir /tmp/arm_connector --output-on-failure
```

The library is also added to the `arm` package with `add_subdirectory(connector)`
in `arm/CMakeLists.txt`, so a normal `colcon build --packages-select arm` builds
and installs it as `libarm_connector.a` + `include/arm_connector/`.

## ROS integration (next step, not in this skeleton)

The wrapper belongs in `arm/src/connector/connector_node.cpp` (to be added) and
should be an `rclcpp_lifecycle` node:

| Transition | Action |
| --- | --- |
| `on_configure` | `transport.open(interface, bitrate)` |
| `on_activate` | `connector.start()` + start the poll/tick timer |
| `on_deactivate` | `connector.stop()` |
| `on_cleanup` | `transport.close()` |

It then feeds `EndpointModule::setConnectorConnected(connector.connected())`
(which lives in the arm controller, see `arm/src/system/endpoint_module.cpp`) so
the `arm/endpoint/state` topic reflects the real CAN link instead of the dummy
flag. The actuator-specific `Ctrl` payloads are added by implementing
`ActuatorControl` for the actuator's `DevTypeId` and registering it in
`ActuatorControlRegistry`.

## TODOs (deliberately left for the implementation phase)

- **Bitrate**: `CanTransport::open()` records the bitrate but does not configure
  the kernel interface (`ip link set can0 type can bitrate ...` is external).
- **Descriptor**: vendor/product/serial are not on the wire yet; classic CAN
  only carries `DevTypeId` + protocol version. Add a multi-frame descriptor
  exchange or move to CAN FD if the full descriptor is required.
- **ACK correlation**: ACKs are matched by the acked command byte. Match by
  sequence too if duplicate/stale frames matter on the bus.
- **Discovery addressing**: the skeleton sends the probe to
  `Config::discovery_can_id`; a real bus may need per-port filtering or a
  multi-actuator discovery sweep.
- **Ctrl payloads**: no actuator family implements `ActuatorControl` yet.
- **Error counters / bus-off recovery**: listen for SocketCAN error frames.
- **Tests**: only the loopback smoke test exists; add state-machine timeout
  tests and a mock-actuator fixture.
