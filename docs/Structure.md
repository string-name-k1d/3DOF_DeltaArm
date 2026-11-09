# Arm-Actuator Interface

Layers:

- Actuator Control + ROS lifecycle
- Discover + Transport
- Descriptor

## Arm Kinematics

## Arm-Actuator Connection Protocol

Abstract

1. Arm-side Connector always initiates interaction
2. Arm-side is responsible of maintaining connection status/handling disconnections
3. Every Message should be responded with ACK
4. The set of Commands should be supported by both commands

### Commands

- Discovery/Disconnect
- Ping
- ACK
- Ctrl

### Arm-Connector States

### Actuator States

- Disconnected
  - Receive Discovery Probe (to Handshake)
- Handshake
  1. send DevTypeId (as ACK)
  2. wait for ReqConnection
  3. send Ready (ACK) -> Mounting
- Mounting
  1. wait for arm connector Ready
  2. send Ping to verify connection
- Unmounting

- Connected
  1. ping every 500ms

> Every wait should be equipped with an appropriate timeout

## Actuator Control Protocol

Should be custom to actuator, identified by DevTypeId
