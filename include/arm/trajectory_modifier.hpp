#ifndef arm__TRAJECTORY_MODIFIER_HPP_
#define arm__TRAJECTORY_MODIFIER_HPP_

#include <algorithm>
#include <cmath>

#include "arm/delta_arm.hpp"

namespace DeltaArm {

/// Straight-line endpoint modifier with speed and acceleration limits.
class EndpointTrajectoryModifier
{
public:
  void reset(const Vec3 & start, const Vec3 & goal, double max_speed, double max_accel)
  {
    start_ = start;
    goal_ = goal;
    max_speed_ = max_speed > 0.0 ? max_speed : 0.0;
    max_accel_ = max_accel > 0.0 ? max_accel : 0.0;
    travelled_ = 0.0;
    speed_ = 0.0;

    const double dx = static_cast<double>(goal.x) - start.x;
    const double dy = static_cast<double>(goal.y) - start.y;
    const double dz = static_cast<double>(goal.z) - start.z;
    distance_ = std::sqrt(dx * dx + dy * dy + dz * dz);
    done_ = distance_ < 1e-6 || max_speed_ == 0.0 || max_accel_ == 0.0;
    if (done_) travelled_ = distance_;
  }

  Vec3 update(double dt)
  {
    if (done_ || !(dt > 0.0)) return setpoint();

    const double remaining = distance_ - travelled_;
    const double stop_distance = speed_ * speed_ / (2.0 * max_accel_);
    const double accel = stop_distance >= remaining ? -max_accel_ : max_accel_;
    speed_ = std::max(0.0, std::min(max_speed_, speed_ + accel * dt));

    double step = speed_ * dt;
    if (step >= remaining) {
      step = remaining;
      speed_ = 0.0;
      done_ = true;
    }
    travelled_ += step;
    return setpoint();
  }

  bool done() const { return done_; }

private:
  Vec3 setpoint() const
  {
    if (distance_ < 1e-6) return goal_;
    const double alpha = std::max(0.0, std::min(1.0, travelled_ / distance_));
    return {
      static_cast<float>(start_.x + alpha * (static_cast<double>(goal_.x) - start_.x)),
      static_cast<float>(start_.y + alpha * (static_cast<double>(goal_.y) - start_.y)),
      static_cast<float>(start_.z + alpha * (static_cast<double>(goal_.z) - start_.z)),
    };
  }

  Vec3 start_{0.0f, 0.0f, 0.0f};
  Vec3 goal_{0.0f, 0.0f, 0.0f};
  double distance_ = 0.0;
  double travelled_ = 0.0;
  double speed_ = 0.0;
  double max_speed_ = 0.0;
  double max_accel_ = 0.0;
  bool done_ = true;
};

/// Per-motor speed and acceleration limiter for commanded angles.
class MotorAccelLimiter
{
public:
  void reset(const float angles_deg[3])
  {
    for (int i = 0; i < 3; ++i) {
      last_[i] = angles_deg[i];
      velocity_[i] = 0.0;
    }
    initialized_ = true;
  }

  void limit(const float desired[3], double dt, double max_speed, double max_accel, float out[3])
  {
    if (!initialized_) reset(desired);
    if (!(dt > 0.0) || max_speed <= 0.0 || max_accel <= 0.0) {
      for (int i = 0; i < 3; ++i) out[i] = desired[i];
      return;
    }

    for (int i = 0; i < 3; ++i) {
      const double remaining = static_cast<double>(desired[i]) - last_[i];
      const double abs_remaining = std::fabs(remaining);
      if (abs_remaining < 1e-9) {
        out[i] = desired[i];
        velocity_[i] = 0.0;
        continue;
      }

      const double direction = remaining > 0.0 ? 1.0 : -1.0;
      double signed_velocity = velocity_[i] * direction;
      if (signed_velocity < 0.0) signed_velocity = 0.0;

      const double stop_distance = signed_velocity * signed_velocity / (2.0 * max_accel);
      const double accel = stop_distance >= abs_remaining ? -max_accel : max_accel;
      signed_velocity = std::max(0.0, std::min(max_speed, signed_velocity + accel * dt));

      double step = signed_velocity * dt;
      if (step >= abs_remaining) {
        step = abs_remaining;
        signed_velocity = 0.0;
      }

      out[i] = static_cast<float>(last_[i] + direction * step);
      last_[i] = out[i];
      velocity_[i] = direction * signed_velocity;
    }
  }

private:
  double last_[3] = {0.0, 0.0, 0.0};
  double velocity_[3] = {0.0, 0.0, 0.0};
  bool initialized_ = false;
};

}  // namespace DeltaArm

#endif  // arm__TRAJECTORY_MODIFIER_HPP_
