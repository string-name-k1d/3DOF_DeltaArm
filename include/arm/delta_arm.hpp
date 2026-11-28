#ifndef arm__DELTA_ARM_HPP_
#define arm__DELTA_ARM_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace DeltaArm
{

/// Task-space point in metres.
struct Vec3
{
  float x;
  float y;
  float z;
};

/// Per-limb mechanism geometry and travel limits; see delta_arm_skill/kinematics.md.
struct ArmMechConfig
{
  float base_radius = 100.0f;
  float platform_radius = 32.5f;
  float upper_arm_len = 120.0f;
  float lower_arm_len = 240.0f;

  float servo_radius = 57.65f;
  float servo_z = -22.5f;
  float upper_rod_len = 60.0f;
  float servo_rod_len = 35.0f;
  float arm_attach_dist = 68.5f;
  float arm_attach_offset = 20.5f;

  float motor_angle_min = 0.0f;
  float motor_angle_max = 145.0f;
  float arm_angle_min = 0.0f;
  float arm_angle_max = 3.14159265358979323846f;

  float plane_angle = 0.0f;
  float home_offset = 0.0f;

  bool bypass_reachability = false;
  bool raw_motor_clamp = true;
};

/// Motor state bookkeeping.
class Motor
{
public:
  Motor();
  explicit Motor(float start_pos);

  void set_tar_pos(float pos);

  float start_pos;
  float cur_pos;
  float tar_pos;
  int servo_id;
};

/// Outcome of a task-space target request.
struct TargetResult
{
  bool reached = true;
  std::string reason;
};

/// Delta-arm kinematics and state.
class Arm
{
public:
  explicit Arm(float start_angles[3] = nullptr);
  ~Arm();

  void init();
  void set_geometry(const std::vector<ArmMechConfig> & configs);

  TargetResult set_tar_pos(float x, float y, float z);
  Vec3 get_tar_pos() const;
  Vec3 get_cur_pos() const;

  void get_motor_current(float out[3]) const;
  void get_motor_targets(float out[3]) const;
  void set_motor_targets(const float angles_deg[3]);

  float arm_angle_from_motor_deg(float motor_deg, int leg) const;
  bool get_linkage_band(int leg, float & lo_deg, float & hi_deg) const;
  bool try_arm_from_motor_deg(float motor_deg, int leg, float & arm_out) const;
  bool linkage_motor_span(int leg, float & motor_lo_deg, float & motor_hi_deg) const;

  void stop();
  void apply();
  void set_motor_feedback(const float angles_deg[3]);

  static constexpr float kReachableTolerance = 1e-3f;

private:
  void init_default_geometry();
  void compute_linkage_bands();

  void ik_stage1(const Vec3 & target);
  float ik_stage2(float theta, int leg);

  float motor_from_arm(float theta_arm, int leg) const;
  float motor_from_arm_unchecked(float theta_arm, int leg) const;
  float arm_from_motor(float motor_rad, int leg) const;
  float nearest_closeable_arm_angle(float theta_arm, int leg) const;

  void compute_ik(const Vec3 & target);
  Vec3 forward_kinematics(const float angles_deg[3]) const;

  std::vector<ArmMechConfig> configs_;

  float stage1_thetas_[3];
  float band_lo_[3];
  float band_hi_[3];
  bool band_valid_[3];

  Motor motors_[3];

  Vec3 tar_pos_;
  Vec3 cur_pos_;

  float start_angles_[3];
  float tar_angles_[3];
  float cur_angles_[3];
};

}  // namespace DeltaArm

#endif  // arm__DELTA_ARM_HPP_
