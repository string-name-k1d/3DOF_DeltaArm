#ifndef arm__MOTOR_DRIVER_HPP_
#define arm__MOTOR_DRIVER_HPP_

#include <cstdint>
#include <string>

#include "FashionStar_UartServo.h"
#include "FashionStar_UartServoProtocol.h"

namespace DeltaArmDriver
{

/**
 * @brief Query "types" shared with the ROS service contract
 * (arm/srv/MotorParamQuery.srv). request_type selects what to read;
 * the returned response_type mirrors it unless remapped inside the driver.
 */
enum QueryType : int32_t
{
  QUERY_ANGLE       = 0,  ///< joint angle (deg)          - e.g. request 0 / response 0
  QUERY_VOLTAGE     = 1,  ///< bus voltage (mV)
  QUERY_CURRENT     = 2,  ///< servo current (mA)
  QUERY_POWER       = 3,  ///< servo power (mW)
  QUERY_TEMPERATURE = 4,  ///< servo temperature (deg. C)
  QUERY_PING        = 5,  ///< ping / online flag (0/1)
};

/**
 * @brief Per-motor actuation configuration.
 *
 * The FashionStar UART-servos are angle servos; their "angle" is the raw
 * output-shaft position in degrees. The values here map that physical range
 * onto the delta-arm's joint-angle convention ("control outputs") and bound
 * the trajectory profiling used by setAngle():
 *
 *   - angle_min/angle_max : physical position limits (deg). Control outputs
 *     are clamped into this window before they reach the servo. For this arm,
 *     0 deg = limb fully extended (away from the centre), 145 deg = fully
 *     retracted.
 *   - install_offset     : physical angle at which the servo is installed
 *     w.r.t. the arm's zero/home pose. A commanded joint angle is sent as
 *     `joint + install_offset`, and read-backs are reported as
 *     `servo_angle - install_offset`, so the driver's "joint angle" interface
 *     stays in the arm's kinematics frame. Wiring `start_angles` here makes
 *     it the per-motor installation offset.
 *   - max_speed         : velocity limit (deg/s) for the servo's on-board
 *     trajectory profiling (converted to a move interval by the SDK).
 */
struct MotorConfig
{
  float angle_min[3]      = {0.0f, 0.0f, 0.0f};          // physical limits (deg)
  float angle_max[3]      = {145.0f, 145.0f, 145.0f};    // physical limits (deg)
  float install_offset[3] = {0.0f, 0.0f, 0.0f};          // servo install offset (deg)
  float max_speed         = 100.0f;                      // velocity limit (deg/s)
};

/**
 * @brief Tight wrapper around the FashionStart UART-servo C++ SDK.
 *
 * Holds one FSUS_Protocol + three FSUS_Servo (one per delta-arm actuator) and
 * exposes a small, ROS-friendly surface:
 *   - set_target_angle(s)/set_target_angles : drive the motors
 *   - query(...)                            : generic "same interface" request
 */
class MotorDriver
{
public:
  /**
   * @param motor_ids binary ids of the three delta-arm servos.
   * @param config    per-motor actuation configuration (limits / offset).
   */
  MotorDriver(const std::string & port_name, uint32_t baudrate,
              const int motor_ids[3],
              const MotorConfig & config = MotorConfig{});
  ~MotorDriver();

  /// Open port, construct + ping + sync the three servos.
  bool init();
  /// Best-effort cleanup (skeleton).
  void close();

  bool   ping(int motor_index, bool * online);
  double query_angle(int motor_index);
  uint16_t queryVoltage(int motor_index);
  uint16_t queryCurrent(int motor_index);
  uint16_t queryPower(int motor_index);
  uint16_t queryTemperature(int motor_index);

  /// Generic query: fills *response_type (mirror) and *value (measured).
  bool query(int motor_index, int32_t request_type,
             int32_t * response_type, int32_t * value);

  bool set_target_angle(int motor_index, float angle_deg);
  bool set_target_angles(const float * angles, int count);

 private:
  std::string port_name_;
  uint32_t baudrate_;
  fsuservo::FSUS_Protocol * protocol_;
  fsuservo::FSUS_Servo   * servos_[3];

  // Per-motor actuation configuration (copied from MotorConfig).
  float angle_min_[3];
  float angle_max_[3];
  float install_offset_[3];
  float max_speed_;

  /// @brief Clamp a joint-angle command into the physical window.
  float clamp_angle(int motor_index, float angle_deg) const;
};

}  // namespace DeltaArmDriver

#endif  // arm__MOTOR_DRIVER_HPP_