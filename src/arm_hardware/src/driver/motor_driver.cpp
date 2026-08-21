#include "arm_hardware/motor_driver.hpp"

namespace DeltaArmDriver
{

// ---------------------------------------------------------------------------
// Construction / destruction
// ---------------------------------------------------------------------------
MotorDriver::MotorDriver(const std::string & port, uint32_t baudrate,
                         const int motor_ids[3])
  : port_name_(port), baudrate_(baudrate), protocol_(nullptr), servos_{nullptr, nullptr, nullptr}
{
  const itas109::BaudRate br = static_cast<itas109::BaudRate>(baudrate_);
  protocol_ = new fsuservo::FSUS_Protocol(port_name_, br);

  for (int i = 0; i < 3; ++i) {
    servos_[i] = new fsuservo::FSUS_Servo(static_cast<fsuservo::FSUS_SERVO_ID_T>(motor_ids[i]),
                                          protocol_);
  }
}

MotorDriver::~MotorDriver()
{
  close();
  for (int i = 0; i < 3; ++i) {
    delete servos_[i];
    servos_[i] = nullptr;
  }
  delete protocol_;
  protocol_ = nullptr;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
bool MotorDriver::init()
{
  // FSUS_Protocol opens the serial transport on construction. init() then
  // pings each servo and syncs its current angle (FSUS_Servo::init()).
  bool ok = true;
  for (int i = 0; i < 3; ++i) {
    if (servos_[i] == nullptr) continue;
    servos_[i]->init();
    ok = ok && servos_[i]->isOnline;
  }
  return ok;
}

void MotorDriver::close()
{
  // On some firmware it is useful to release torque before closing the port.
  // TODO(user): servos_[i]->setDamping(0); if desired.
  // The CSerialPort transport is enclosed inside FSUS_Protocol; it will be
  // released with the protocol object. Extend here if an explicit close is
  // required by the custom FIT invocation.
}

// ---------------------------------------------------------------------------
// Queries (generic "same interface" model)
// ---------------------------------------------------------------------------
bool MotorDriver::ping(int motor_index, bool * online)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return false;
  *online = servos_[motor_index]->ping();
  return *online;
}

double MotorDriver::query_angle(int motor_index)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return 0.0;
  return servos_[motor_index]->queryAngle();
}

uint16_t MotorDriver::queryVoltage(int motor_index)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return 0;
  return servos_[motor_index]->queryVoltage();
}

uint16_t MotorDriver::queryCurrent(int motor_index)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return 0;
  return servos_[motor_index]->queryCurrent();
}

uint16_t MotorDriver::queryPower(int motor_index)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return 0;
  return servos_[motor_index]->queryPower();
}

uint16_t MotorDriver::queryTemperature(int motor_index)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return 0;
  return servos_[motor_index]->queryTemperature();
}

bool MotorDriver::query(int motor_index, int32_t request_type,
                        int32_t * response_type, int32_t * value)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return false;

  *response_type = request_type;   // mirror by default
  *value = 0;

  switch (QueryType(request_type)) {
    case QUERY_ANGLE:
      *value = static_cast<int32_t>(query_angle(motor_index));
      return true;
    case QUERY_VOLTAGE:
      *value = static_cast<int32_t>(queryVoltage(motor_index));
      return true;
    case QUERY_CURRENT:
      *value = static_cast<int32_t>(queryCurrent(motor_index));
      return true;
    case QUERY_POWER:
      *value = static_cast<int32_t>(queryPower(motor_index));
      return true;
    case QUERY_TEMPERATURE:
      *value = static_cast<int32_t>(queryTemperature(motor_index));
      return true;
    case QUERY_PING:
    {
      bool online = false;
      *value = (ping(motor_index, &online)) ? 1 : 0;
      return true;
    }
    default:
      *response_type = -1;   // unknown request
      return false;
  }
}

// ---------------------------------------------------------------------------
// Actuation
// ---------------------------------------------------------------------------
bool MotorDriver::set_target_angle(int motor_index, float angle_deg)
{
  if (motor_index < 0 || motor_index > 2 || servos_[motor_index] == nullptr) return false;
  // setAngle() works in "real" degrees of the servo output shaft.
  servos_[motor_index]->setAngle(angle_deg);
  return true;
}

bool MotorDriver::set_target_angles(const float * angles, int count)
{
  bool ok = true;
  for (int i = 0; i < count && i < 3; ++i) {
    ok = set_target_angle(i, angles[i]) && ok;
  }
  return ok;
}

}  // namespace DeltaArmDriver