#include "arm/control_augmenter.hpp"

#include <algorithm>
#include <cmath>

namespace DeltaArmRos
{

void ControlAugmenter::configure(
  const std::array<double, 3> & offset_deg,
  bool enable_feedforward,
  double max_delta_deg)
{
  offset_deg_ = offset_deg;
  enable_feedforward_ = enable_feedforward;
  max_delta_deg_ = max_delta_deg;
  reset();
}

void ControlAugmenter::reset()
{
  prev_out_.fill(0.0f);
  have_prev_ = false;
  last_msg_time_ = std::chrono::steady_clock::time_point{};
}

// ---------------------------------------------------------------------------
// Pipeline
// ---------------------------------------------------------------------------
arm::msg::MotorTargets ControlAugmenter::augment(const arm::msg::MotorTargets & raw)
{
  arm::msg::MotorTargets out = raw;  // header (stamp/frame) passes through

  float angles[3] = {raw.angles[0], raw.angles[1], raw.angles[2]};

  const auto now_t = std::chrono::steady_clock::now();
  const double dt = have_prev_
    ? std::chrono::duration<double>(now_t - last_msg_time_).count()
    : 0.0;
  last_msg_time_ = now_t;

  apply_offset(angles);
  apply_feedforward(angles, prev_out_.data(), dt);
  clamp_limits(angles);

  for (int i = 0; i < 3; ++i) {
    out.angles[i] = angles[i];
    prev_out_[i] = angles[i];
  }
  have_prev_ = true;
  return out;
}

void ControlAugmenter::apply_offset(float angles[3]) const
{
  for (int i = 0; i < 3; ++i) {
    angles[i] += static_cast<float>(offset_deg_[i]);
  }
}

void ControlAugmenter::apply_feedforward(float angles[3], const float prev[3], double dt)
{
  if (!enable_feedforward_) {
    return;  // disabled: identity
  }

  // TODO(user): implement the feedforward contribution. Available inputs:
  //   angles : the offset-corrected command (deg) - modify in place
  //   prev   : the previous OUTPUT of this pipeline (deg)
  //   dt     : seconds since the previous message (0 on the first message)
  //
  // Typical implementations:
  //   * velocity/acceleration shaping:  a = (angles - prev) / dt
  //   * gravity/friction compensation:  add a pose-dependent term
  //   * backlash pre-load / tensioning: add a small constant bias
  //   * PD lead term on the servo error
  //
  // The robot is UNCHANGED until this body is filled in.
  (void)angles;
  (void)prev;
  (void)dt;
}

void ControlAugmenter::clamp_limits(float angles[3])
{
  if (max_delta_deg_ > 0.0 && have_prev_) {
    const float max_delta = static_cast<float>(max_delta_deg_);
    for (int i = 0; i < 3; ++i) {
      const float delta = std::clamp(angles[i] - prev_out_[i], -max_delta, max_delta);
      angles[i] = prev_out_[i] + delta;
    }
  }

  // TODO(user): add hard position limits here if the driver's angle_min /
  // angle_max should be enforced earlier (the driver already clamps).
}

}  // namespace DeltaArmRos