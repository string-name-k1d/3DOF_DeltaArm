#ifndef arm__CONTROL_AUGMENTER_HPP_
#define arm__CONTROL_AUGMENTER_HPP_

#include <array>
#include <chrono>

#include "arm/msg/motor_targets.hpp"

namespace DeltaArmRos
{

/**
 * @brief Control-augmentation logic for the delta arm (LOGIC ONLY - not a node).
 *
 * No ROS node, publishers or subscriptions here: this class is the pure
 * "offset -> feedforward -> clamp" pipeline, owned by the controller node
 * (src/controller/delta_arm_controller_node.cpp), which runs it on every
 * MotorTargets message before publishing:
 *
 *   arm_controller (IK output) --arm/motor_targets--> arm_motor_driver
 *                       ^
 *                       +- control augmenter (offset / feedforward / clamp
 *                          applied in place to every published message)
 *
 * The driver therefore consumes the already-augmented stream on
 * arm/motor_targets directly (its default `targets_topic`); there is no
 * separate relay node or arm/motor_targets_cmd topic any more.
 *
 * Use this class to add control-level features without touching the kinematic
 * solver or the driver, for example:
 *   * per-limb target OFFSETS (calibration / installation correction);
 *   * FEEDFORWARD terms (velocity/acceleration shaping, gravity or friction
 *     compensation, tensioning the linkage against backlash);
 *   * rate limiting / slew shaping and safety clamps;
 *   * lead/lag compensation between the command and the measured servo angle.
 *
 * The pipeline is deliberately explicit and fully populated by default
 * (identity): every hook is a no-op until implemented, so enabling the class
 * cannot change the robot's behaviour before the TODOs below are filled.
 *
 * Parameters (declared by the owning controller node, config/arm_params.yaml,
 * block `arm_controller`):
 *   offset_deg          ([0,0,0])                per-limb constant offset (deg)
 *   enable_feedforward  (false)                  master switch for the FF hook
 *   max_delta_deg       (0.0)                    optional per-call slew clamp;
 *                                                0 = disabled
 */
class ControlAugmenter
{
public:
  ControlAugmenter() = default;

  /// Set the pipeline parameters. Call once from the owning node's
  /// declareParams() before any augment() call.
  void configure(
    const std::array<double, 3> & offset_deg,
    bool enable_feedforward,
    double max_delta_deg);

  /// Drop the per-message state (previous output + timing). The next augment()
  /// behaves as a "first message": dt = 0, no slew clamping. The controller
  /// calls this on emergency stop so the zeroed output is not slew-limited.
  void reset();

  /// Full augmentation pipeline: offset -> feedforward -> clamp.
  /// `raw` is the controller's IK output (motor angles, degrees); the returned
  /// message is a copy with the augmented angles (header passes through).
  arm::msg::MotorTargets augment(const arm::msg::MotorTargets & raw);

private:
  /// Hook: add the configured per-limb constant offset (installation /
  /// calibration correction). Implemented.
  void apply_offset(float angles[3]) const;

  /// Hook: feedforward contribution (velocity/acceleration shaping, gravity or
  /// friction compensation, backlash pre-load, ...).
  ///
  /// TODO(user): implement. `prev`/`dt` are provided so the hook can
  /// differentiate the command stream; the reference-integrated state is not
  /// kept here. Default: no-op (returns before touching `angles`).
  void apply_feedforward(float angles[3], const float prev[3], double dt);

  /// Hook: final safety clamp (position limits, slew limit, ...).
  /// Implemented: slew clamp when max_delta_deg_ > 0, else passthrough.
  void clamp_limits(float angles[3]);

  std::array<double, 3> offset_deg_ = {0.0, 0.0, 0.0};
  bool enable_feedforward_ = false;
  double max_delta_deg_ = 0.0;

  // Previous output (needed by the slew clamp and available to the
  // feedforward hook) + timestamp of the last processed message.
  std::array<float, 3> prev_out_ = {0.0f, 0.0f, 0.0f};
  bool have_prev_ = false;
  std::chrono::steady_clock::time_point last_msg_time_;
};

}  // namespace DeltaArmRos

#endif  // arm__CONTROL_AUGMENTER_HPP_