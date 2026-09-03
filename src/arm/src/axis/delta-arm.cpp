#include "arm/delta_arm.hpp"

#include <cmath>
#include <stdexcept>

namespace DeltaArm
{

// ---------------------------------------------------------------------------
// Motor
// ---------------------------------------------------------------------------
Motor::Motor() : Motor(0.0f)
{
}

Motor::Motor(float start_pos)
  : start_pos(start_pos), cur_pos(start_pos), tar_pos(start_pos), servo_id(0)
{
}

void Motor::set_tar_pos(float pos)
{
  tar_pos = pos;
}

namespace
{

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kSqrt3 = 1.7320508075688772f;
constexpr float kTan30 = 0.5773502691896258f;  // 1/sqrt(3)
constexpr float kCos120 = -0.5f;
constexpr float kSin120 = 0.8660254037844386f;  // sqrt(3)/2

/**
 * Solves the classic per-limb delta IK (DeltaKin, R.L. Williams II /
 * tinkersprojects delta_calcAngleYZ formulation).
 *
 * Takes the end-effector position (x0, y0, z0) in the limb's coordinate frame
 * (which has been rotated so the limb lies in the yz-plane) and the geometry
 * (base triangle side `f`, effector triangle side `e`, upper arm `rf`,
 * lower rod `re`). Returns the upper-arm angle in DEGREES (the sign/orientation
 * follows the reference implementation). Throws if the point is unreachable.
 *
 * Note: z must be non-zero for the division by z0 to be safe.
 */
float delta_calc_angle_yz(float x0, float y0, float z0,
                          float f, float e, float rf, float re)
{
  // Center-to-edge shifts for the two equilateral triangles.
  const float y1 = -0.5f * kTan30 * f;  // -f/2 * tan(30)  (base joint offset)
  y0 -= 0.5f * kTan30 * e;              // shift the effector center to its edge

  // Line through the two intersection points: z = a + b*y
  const float a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) /
                  (2.0f * z0);
  const float b = (y1 - y0) / z0;

  // Discriminant of the circle-intersection quadratic.
  const float d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf);
  if (d < 0.0f) {
    throw std::invalid_argument("Unreachable delta target (discriminant < 0)");
  }

  // Choose the outer (valid) intersection point.
  const float yj = (y1 - a * b - std::sqrt(d)) / (b * b + 1.0f);
  const float zj = a + b * yj;

  // Upper-arm angle (degrees) relative to the base.
  float theta = std::atan2(-zj, (y1 - yj)) * kRadToDeg;
  if (yj > y1) {
    theta += 180.0f;
  }
  return theta;
}

}  // namespace

// ---------------------------------------------------------------------------
// Arm
// ---------------------------------------------------------------------------
Arm::Arm(float start_angles[3])
{
  for (int i = 0; i < 3; ++i) {
    start_angles_[i] = (start_angles != nullptr) ? start_angles[i] : 0.0f;
    motors_[i] = Motor(start_angles_[i]);
    motors_[i].servo_id = i;
    cur_angles_[i] = start_angles_[i];
    tar_angles_[i] = start_angles_[i];
  }
  tar_pos_ = {0.0f, 0.0f, 0.0f};
  cur_pos_ = {0.0f, 0.0f, 0.0f};

  init_default_geometry();
}

Arm::~Arm()
{
}

void Arm::init()
{
  for (int i = 0; i < 3; ++i) {
    start_angles_[i] = motors_[i].start_pos;
    motors_[i].cur_pos = start_angles_[i];
    motors_[i].tar_pos = start_angles_[i];
    cur_angles_[i] = start_angles_[i];
    tar_angles_[i] = start_angles_[i];
  }
  cur_pos_ = {0.0f, 0.0f, 0.0f};
  tar_pos_ = {0.0f, 0.0f, 0.0f};
}

void Arm::init_default_geometry()
{
  constexpr float kTwoPiOver3 = 2.0943951023931953f;  // 120 deg
  configs_.clear();
  for (int leg = 0; leg < 3; ++leg) {
    ArmMechConfig c;
    c.base_radius = 150.0f;      // mm (circumradius of the base triangle)
    c.platform_radius = 60.0f;   // mm (circumradius of the effector triangle)
    c.upper_arm_len = 160.0f;    // mm
    c.lower_arm_len = 320.0f;    // mm
    c.plane_angle = leg * kTwoPiOver3;
    c.home_offset = 0.0f;
    configs_.push_back(c);
  }
}

void Arm::set_geometry(const std::vector<ArmMechConfig> & configs)
{
  if (configs.size() != 3) {
    throw std::invalid_argument("set_geometry requires exactly 3 limb configs");
  }
  configs_ = configs;
}

// ---------------------------------------------------------------------------
// Two-stage inverse kinematics (classic 3-DOF delta)
// ---------------------------------------------------------------------------

/**
 * STAGE 1 - task-space target -> per-limb plane orientation.
 *
 * For each limb, rotate the 3D target into that limb's local frame (so the
 * limb lies in its yz-plane) and solve the classic delta IK. The resulting
 * upper-arm angle (stored in stage1_thetas_[]) is the per-limb orientation
 * angle that stage 2 maps to a motor angle.
 */
void Arm::ik_stage1(const Vec3 & target)
{
  // Convert metres -> millimetres (the geometry uses mm).
  const float x = target.x * 1000.0f;
  const float y = target.y * 1000.0f;
  const float z = target.z * 1000.0f;

  const float f = configs_[0].base_radius * kSqrt3;      // base triangle side
  const float e = configs_[0].platform_radius * kSqrt3;  // effector triangle side

  for (int leg = 0; leg < 3; ++leg) {
    // Rotate the target into the limb's frame (the limb's plane becomes the
    // yz-plane). The angle for leg 0 is 0; legs 1 & 2 at +-120 deg.
    const ArmMechConfig & c = configs_[leg];
    const float ca = std::cos(c.plane_angle);
    const float sa = std::sin(c.plane_angle);
    const float rx = x * ca + y * sa;
    const float ry = -x * sa + y * ca;

    // Upper-arm angle in degrees for this limb.
    const float theta_deg =
      delta_calc_angle_yz(rx, ry, z, f, e, c.upper_arm_len, c.lower_arm_len);
    stage1_thetas_[leg] = theta_deg * kDegToRad;
  }
}

/**
 * STAGE 2 - limb plane orientation -> motor joint angle (degrees).
 *
 * Applies the limb's home/calibration offset and converts to the driver's
 * degrees convention.
 */
float Arm::ik_stage2(float theta, int leg)
{
  return (theta * kRadToDeg + configs_[leg].home_offset * kRadToDeg);
}

void Arm::compute_ik(const Vec3 & target)
{
  ik_stage1(target);
  for (int leg = 0; leg < 3; ++leg) {
    tar_angles_[leg] = ik_stage2(stage1_thetas_[leg], leg);
    motors_[leg].set_tar_pos(tar_angles_[leg]);
  }
}

/**
 * Forward kinematics: given the three upper-arm angles (degrees), compute the
 * end-effector position (metres) by intersecting the three spheres of radius
 * `lower_arm_len` centered on the three arm ends (classic delta direct
 * kinematics, DeltaKin reference).
 */
Vec3 Arm::forward_kinematics(const float angles_deg[3]) const
{
  // Geometry (triangle sides from the same radii as IK).
  const float f = configs_[0].base_radius * kSqrt3;
  const float e = configs_[0].platform_radius * kSqrt3;
  const float re = configs_[0].lower_arm_len;

  // Offset parameter: the distance from the robot centre to a joint line.
  const float t = (f - e) * kTan30 / 2.0f;

  // Arm endpoints J1, J2, J3 (classic direct-kinematics construction).
  const float a1 = angles_deg[0] * kDegToRad;
  const float a2 = angles_deg[1] * kDegToRad;
  const float a3 = angles_deg[2] * kDegToRad;

  const float y1 = -(t + configs_[0].upper_arm_len * std::cos(a1));
  const float z1 = -configs_[0].upper_arm_len * std::sin(a1);

  const float y2p = t + configs_[1].upper_arm_len * std::cos(a2);
  const float x2 = y2p * kSin120;
  const float y2 = y2p * kCos120;
  const float z2 = -configs_[1].upper_arm_len * std::sin(a2);

  const float y3p = t + configs_[2].upper_arm_len * std::cos(a3);
  const float x3 = -y3p * kSin120;
  const float y3 = y3p * kCos120;
  const float z3 = -configs_[2].upper_arm_len * std::sin(a3);

  // Denominator for the linear solve.
  const float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  const float w1 = y1 * y1 + z1 * z1;
  const float w2 = x2 * x2 + y2 * y2 + z2 * z2;
  const float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // x = (a1*z + b1)/dnm
  const float c1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  const float d1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0f;

  // y = (a2*z + b2)/dnm
  const float c2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  const float d2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0f;

  // a*z^2 + b*z + cc = 0
  const float aa = c1 * c1 + c2 * c2 + dnm * dnm;
  const float bb = 2.0f * (c1 * d1 + c2 * (d2 - y1 * dnm) - z1 * dnm * dnm);
  const float cc =
    (d2 - y1 * dnm) * (d2 - y1 * dnm) + d1 * d1 + dnm * dnm * (z1 * z1 - re * re);

  const float disc = bb * bb - 4.0f * aa * cc;
  if (disc < 0.0f) {
    return {0.0f, 0.0f, 0.0f};
  }

  const float z0 = -0.5f * (bb + std::sqrt(disc)) / aa;
  const float x0 = (c1 * z0 + d1) / dnm;
  const float y0 = (c2 * z0 + d2) / dnm;

  Vec3 out;
  out.x = x0 * 0.001f;  // mm -> m
  out.y = y0 * 0.001f;
  out.z = z0 * 0.001f;
  return out;
}

// ---------------------------------------------------------------------------
// Public state interface
// ---------------------------------------------------------------------------
void Arm::set_tar_pos(float x, float y, float z)
{
  tar_pos_ = {x, y, z};
  try {
    compute_ik(tar_pos_);
  } catch (const std::exception &) {
    // Unreachable target: leave the previous targets intact.
  }
}

Vec3 Arm::get_tar_pos() const
{
  return tar_pos_;
}

Vec3 Arm::get_cur_pos() const
{
  return cur_pos_;
}

void Arm::get_motor_current(float out[3]) const
{
  for (int i = 0; i < 3; ++i) out[i] = cur_angles_[i];
}

void Arm::get_motor_targets(float out[3]) const
{
  for (int i = 0; i < 3; ++i) out[i] = motors_[i].tar_pos;
}

void Arm::stop()
{
  // Emergency halt: cancel any commanded motion and zero the motor outputs.
  for (int i = 0; i < 3; ++i) {
    motors_[i].set_tar_pos(0.0f);
    motors_[i].cur_pos = 0.0f;
    cur_angles_[i] = 0.0f;
    tar_angles_[i] = 0.0f;
  }
  tar_pos_ = {0.0f, 0.0f, 0.0f};
  cur_pos_ = {0.0f, 0.0f, 0.0f};
}

void Arm::apply()
{
  // Promote the commanded targets to the "current" state and refresh the
  // forward-position estimate.
  for (int i = 0; i < 3; ++i) {
    cur_angles_[i] = motors_[i].tar_pos;
    motors_[i].cur_pos = cur_angles_[i];
  }

  cur_pos_ = forward_kinematics(cur_angles_);
}

}  // namespace DeltaArm
