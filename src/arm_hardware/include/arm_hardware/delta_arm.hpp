#ifndef arm_HARDWARE__DELTA_ARM_HPP_
#define arm_HARDWARE__DELTA_ARM_HPP_

#include <cstdint>

namespace DeltaArm
{

/**
 * @brief Task-space vector (meters).
 */
struct Vec3
{
  float x;
  float y;
  float z;
};

/**
 * @brief A single motor joint.
 *
 * Hardware-neutral: this only tracks bookkeeping (start / current / target
 * angle). The physical FashionStar actuator is owned by the motor-driver
 * layer (see include/arm_hardware/motor_driver.hpp). Keeping this decoupled
 * means the controller node can run the kinematics without touching hardware.
 */
class Motor
{
public:
  Motor();                       // start_pos = 0
  explicit Motor(float start_pos);

  void set_tar_pos(float pos);   // sets the commanded target angle (deg)

  float start_pos;
  float cur_pos;
  float tar_pos;
  int servo_id;                  // index/id of the physical servo (0..2)
};

/**
 * @brief 3-DOF Delta arm - kinematics + state bookkeeping only.
 *
 * End-effector motion is produced with a two-stage inverse kinematics:
 *   stage 1 : given the task-space target, solve each limb's plane orientation;
 *   stage 2 : convert each plane orientation into that limb's motor angle.
 */
class Arm
{
public:
  explicit Arm(float start_angles[3] = nullptr);
  ~Arm();

  /// @brief Reset state to the calibrated start angles.
  void init();

  /// @brief Task-space interface (x, y, z in meters).
  void set_tar_pos(float x, float y, float z);
  Vec3 get_tar_pos() const;      ///< last commanded target
  Vec3 get_cur_pos() const;      ///< estimated end-effector pose (from forward estimate)

  /// @brief Motor-space outputs (degrees).
  void get_motor_current(float out[3]) const;
  void get_motor_targets(float out[3]) const;

  /// @brief Emergency halt: cancel pending motion, zero all motor targets.
  void stop();

  /// @brief Called after the driver applied the targets: promote targets ->
  ///        currents and refresh the forward position estimate.
  void apply();

 private:
  /// @brief Two-stage inverse kinematics (skeleton - MATH IMPLEMENTATION REQUIRED).
  void ik_stage1(const Vec3 & target);      ///< task-space -> limb plane orientations
  float ik_stage2(float theta, int leg);   ///< limb plane orientation -> motor angle

  void compute_ik(const Vec3 & target);     ///< runs stage 1 + stage 2, fills tar_angles_

  Motor motors_[3];

  Vec3 tar_pos_;
  Vec3 cur_pos_;

  float start_angles_[3];
  float tar_angles_[3];
  float cur_angles_[3];
};

}  // namespace DeltaArm

#endif  // arm_HARDWARE__DELTA_ARM_HPP_