#include "arm_hardware/delta_arm.hpp"

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

// ---------------------------------------------------------------------------
// Two-stage inverse kinematics (SKELETON - math deliberately left for you)
// ---------------------------------------------------------------------------

/**
 * STAGE 1 - task-space target -> per-limb plane orientation.
 *
 * TODO(user): implement the Delta-arm stage-1 inverse kinematics here.
 * For a 3-legged parallel delta arm, the end-effector position (x, y, z) is
 * closed by three limbs; each limb rotates as a rigid parallelogram held by a
 * single motor. This stage solves the unknown orientation of each limb plane.
 * Store intermediate per-leg results in private members you add (or return
 * them through ik_stage2).
 */
void Arm::ik_stage1(const Vec3 & target)
{
  (void)target;   // placeholder - remove once math is implemented
}

/**
 * STAGE 2 - limb plane orientation -> motor joint angle (degrees).
 *
 * TODO: implement the stage-2 mapping, e.g. motor_angle = f(theta) using the
 * parallelogram link lengths and the motor's calibrated home offset.
 */
float Arm::ik_stage2(float /*theta*/, int /*leg*/)
{
  return 0.0f;    // placeholder
}

void Arm::compute_ik(const Vec3 & target)
{
  // Stage 1 then stage 2, storing the three motor target angles.
  ik_stage1(target);
  for (int leg = 0; leg < 3; ++leg) {
    tar_angles_[leg] = ik_stage2(0.0f, leg);
    motors_[leg].set_tar_pos(tar_angles_[leg]);
  }
}

// ---------------------------------------------------------------------------
// Public state interface
// ---------------------------------------------------------------------------
void Arm::set_tar_pos(float x, float y, float z)
{
  tar_pos_ = {x, y, z};
  compute_ik(tar_pos_);
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

  // TODO: forward kinematics - estimate cur_pos_ from cur_angles_ (the delta
  // arm forward model is the inverse of the two-stage IK above).
}

}  // namespace DeltaArm