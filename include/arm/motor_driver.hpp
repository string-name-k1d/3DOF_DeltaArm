#ifndef arm__MOTOR_DRIVER_HPP_
#define arm__MOTOR_DRIVER_HPP_

#include <cstdint>
#include <string>

#include "FashionStar_UartServo.h"
#include "FashionStar_UartServoProtocol.h"

namespace DeltaArmDriver
{

/// Per-servo travel and installation settings.
struct MotorConfig
{
  float angle_min[3]      = {0.0f, 0.0f, 0.0f};
  float angle_max[3]      = {145.0f, 145.0f, 145.0f};
  float install_offset[3] = {0.0f, 0.0f, 0.0f};
  float max_speed         = 100.0f;
};

/// FashionStar UART bus wrapper for three servos.
class MotorDriver
{
public:


  MotorDriver(const std::string & port_name, uint32_t baudrate,
              const int motor_ids[3],
              const MotorConfig & config = MotorConfig{});
  ~MotorDriver();


  bool init();

  void close();

  bool   ping(int motor_index, bool * online);
  double query_angle(int motor_index);

  bool set_target_angle(int motor_index, float angle_deg);
  bool set_target_angles(const float * angles, int count);

 private:
  std::string port_name_;
  uint32_t baudrate_;
  fsuservo::FSUS_Protocol * protocol_;
  fsuservo::FSUS_Servo   * servos_[3];


  float angle_min_[3];
  float angle_max_[3];
  float install_offset_[3];
  float max_speed_;


  float clamp_angle(int motor_index, float angle_deg) const;
};

}

#endif