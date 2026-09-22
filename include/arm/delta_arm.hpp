#ifndef arm__DELTA_ARM_HPP_
#define arm__DELTA_ARM_HPP_

#include <cstdint>
#include <vector>

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
 * @brief Geometry of one delta-arm limb.
 *
 * A classic 3-DOF delta robot limb (per R.L. Williams II, "The Delta
 * Parallel Robot: Kinematics Solutions", 2016): a servo at the base drives an
 * upper arm of length `upper_arm_len` that sweeps in a vertical plane; a
 * lower (parallelogram) rod of length `lower_arm_len` connects the arm's end
 * to the end-effector platform. The three limbs are spaced at equal angles
 * around the base's central axis.
 *
 * The servo drives the arm through a 4-BAR LINKAGE instead of turning the arm
 * directly: a horn (upper_rod_len) keyed to the servo shaft, a connecting rod
 * (servo_rod_len) to a bracket on the upper arm (arm_attach_dist along the arm
 * axis + arm_attach_offset perpendicular standoff). Stage-2 IK solves that
 * linkage's closed form to map limb angle <-> motor angle.
 *
 * All lengths are in millimetres; the task-space interface (set_tar_pos) uses
 * metres, which are converted internally to millimetres so the geometry keeps
 * clean whole numbers that match the CAD (.sldasm) model.
 */
struct ArmMechConfig
{
  float base_radius = 100.0f;    // horizontal distance of a shoulder pivot from
                                 // the base's central axis (mm)
  float platform_radius = 32.5f; // horizontal distance of a platform joint
                                 // from the end-effector axis (mm)
  float upper_arm_len = 120.0f;  // shoulder -> arm end (elbow) (mm)
  float lower_arm_len = 240.0f;  // arm end -> platform joint rod (mm)

  float servo_radius = 57.65f;   // horizontal distance of a servo output shaft
                                 // from the base's central axis (mm)
  float servo_z = -22.5f;        // servo shaft height BELOW the shoulder plane
                                 // (mm; mounts under the base plate)
  float upper_rod_len = 60.0f;   // servo horn: the short crank on the servo
                                 // shaft (4-bar link `a`, mm)
  float servo_rod_len = 35.0f;   // rigid connecting rod between horn and arm
                                 // bracket (4-bar link `b`, mm)
  float arm_attach_dist = 68.5f; // shoulder -> attach bracket along the arm
                                 // axis (mm; 4-bar link `c` = hypot of this
                                 // with arm_attach_offset)
  float arm_attach_offset = 20.5f; // perpendicular standoff of the rod's
                                   // arm-side ball socket from the arm axis (mm)

  float plane_angle = 0.0f;      // angle (radians) of this limb's vertical
                                 // plane around the base's central axis
  float home_offset = 0.0f;      // motor home/calibration offset (radians)
};

/**
 * @brief A single motor joint.
 *
 * Hardware-neutral: this only tracks bookkeeping (start / current / target
 * angle). The physical FashionStar actuator is owned by the motor-driver
 * layer (see include/arm/motor_driver.hpp). Keeping this decoupled
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

  /// @brief Set the per-limb mechanism geometry (3 configs, one per leg).
  ///        If not called, a default concentric 3-leg configuration is used.
  void set_geometry(const std::vector<ArmMechConfig> & configs);

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

  /// @brief Whether the forward estimate (cur_pos_) tracks the target.
  static constexpr float kReachableTolerance = 1e-3f;

 private:
  /// @brief Build the default 3-leg concentric geometry.
  void init_default_geometry();

  /// @brief Two-stage inverse kinematics.
  void ik_stage1(const Vec3 & target);      ///< task-space -> limb plane orientations
  float ik_stage2(float theta, int leg);   ///< limb plane orientation -> motor angle (deg)

  /// @brief 4-bar linkage solve: arm angle (rad) -> motor angle (rad), using the
  ///        closed form (a=horn, b=rod, c=shoulder->socket, d=servo->shoulder).
  ///        Throws std::invalid_argument when the linkage cannot close.
  float motor_from_arm(float theta_arm, int leg) const;
  /// @brief Inverse 4-bar solve: motor angle (rad) -> arm angle (rad). Monotone
  ///        bisection of the closing band; clamped to the band edges.
  float arm_from_motor(float motor_rad, int leg) const;

  void compute_ik(const Vec3 & target);     ///< runs stage 1 + stage 2, fills tar_angles_

  // Classic delta per-limb geometric solver: given the end-effector position
  // relative to the base, return the limb's required upper-arm angle
  // (radians). Throws if unreachable.
  float delta_calc_angle_deg(float x0, float y0, float z0, const ArmMechConfig & c) const;

  // Estimate the 3D end-effector position from the current SERVO (motor)
  // angles (degrees): each motor angle is first converted back to the limb's
  // upper-arm angle via the 4-bar solve, then the classic delta FK runs.
  Vec3 forward_kinematics(const float angles_deg[3]) const;

  std::vector<ArmMechConfig> configs_;

  float stage1_thetas_[3];  ///< per-limb plane orientations (radians) from stage 1

  Motor motors_[3];

  Vec3 tar_pos_;
  Vec3 cur_pos_;

  float start_angles_[3];
  float tar_angles_[3];
  float cur_angles_[3];
};

}  // namespace DeltaArm

#endif  // arm__DELTA_ARM_HPP_