#ifndef arm__ARM_STATE_CONTROL_HPP_
#define arm__ARM_STATE_CONTROL_HPP_

#include <array>
#include <cstdint>
#include <string>

namespace DeltaArmRos
{

/**
 * @brief High-level state machine for the delta arm (pure logic, no ROS I/O).
 *
 * Three states, mirroring the arm architecture plan:
 *  - kZeroEmergency  estop latched: goals are rejected, motors must be zero.
 *  - kHome           safe resting posture, goals accepted (they move the arm).
 *  - kPositionControl actively tracking/holding a task-space setpoint.
 *
 * The controller node owns an instance and reflects the state on
 * `arm/control_state`; the state itself decides whether a `set_pos` goal is
 * accepted, and estop activation also forces the motor output to zero.
 */
class ArmStateControl
{
public:
  enum class State
  {
    kZeroEmergency,
    kHome,
    kPositionControl
  };

  // Boots in kHome so a freshly started node accepts goals (no latched estop).
  ArmStateControl() = default;

  // Safe resting posture in meters (task space), used when returning home.
  void configureHome(const std::array<double, 3> & home_m);

  // Move to a state. Rejects no-op requests and requests that would release a
  // latched estop (use releaseEmergencyStop for that). Returns true when the
  // state actually changed.
  bool requestState(State state, const std::string & reason = std::string());

  // Latch estop: forces kZeroEmergency and stays there until released.
  bool activateEmergencyStop(const std::string & reason = std::string());

  // Clear the estop latch: returns to kHome.
  bool releaseEmergencyStop(const std::string & reason = std::string());

  // Called when a set_pos goal is accepted; promotes kHome -> kPositionControl.
  // Returns false (and changes nothing) while an estop is latched.
  bool onGoalAccepted();

  // Called when the arm settles back at the home posture.
  bool onHomeReached();

  State getState() const noexcept { return state_; }
  bool isEmergencyActive() const noexcept { return state_ == State::kZeroEmergency; }

  // Only the zero/emergency state blocks new position goals.
  bool acceptsGoals() const noexcept { return state_ != State::kZeroEmergency; }

  const std::array<double, 3> & getHomePose() const noexcept { return home_m_; }
  const std::string & getLastReason() const noexcept { return last_reason_; }
  std::uint64_t getTransitionCount() const noexcept { return transitions_; }

  static const char * toString(State state) noexcept;

private:
  void set(State state, const std::string & reason);

  State state_{State::kHome};
  std::array<double, 3> home_m_{0.0, 0.0, -0.30};
  std::string last_reason_{"initialized in home state"};
  std::uint64_t transitions_{0};
};

}  // namespace DeltaArmRos

#endif  // arm__ARM_STATE_CONTROL_HPP_