#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "arm/delta_arm.hpp"

// ---------------------------------------------------------------------------
// These tests exercise the arm-library plumbing (state bookkeeping + the IK
// pipeline): motor/target bookkeeping, the classic delta two-stage IK vs FK
// round trip, and the 4-bar servo-linkage transmission the stage-2 IK solves.
//
// A large part of this file guards the REACHABLE ENVELOPE. The arm can only be
// in a small region of task space, and the historical bug was that the forward
// kinematics silently clamped a limb's arm angle to 90 deg (or to 0) whenever
// the commanded pose needed more, so the drawn/streamed position disagreed with
// the commanded target by up to 64 mm while the motor angles were perfectly
// correct. These tests assert the two directions of the solve agree.
// ---------------------------------------------------------------------------

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = 0.017453292519943295f;
constexpr float kRadToDeg = 57.29577951308232f;
constexpr float kTwoPiOver3 = 2.0943951023931953f;

// The default assembly (arm/config/arm_params.yaml): horn 60, rod 35.
DeltaArm::ArmMechConfig defaultLimb(int leg)
{
  DeltaArm::ArmMechConfig c;
  c.plane_angle = leg * kTwoPiOver3;
  return c;
}

// The gen0 assembly (arm/config/arm_gen0_params.yaml), which deliberately uses
// the OPPOSITE horn/rod proportions (horn 35, rod 60) and a smaller platform
// radius. The two are separate physical machines, not variants of one.
DeltaArm::ArmMechConfig gen0Limb(int leg)
{
  DeltaArm::ArmMechConfig c;
  c.base_radius = 95.0f;
  c.platform_radius = 43.3f;
  c.servo_radius = 60.0f;
  c.upper_rod_len = 35.0f;
  c.servo_rod_len = 60.0f;
  c.arm_attach_dist = 52.0f;
  c.plane_angle = leg * kTwoPiOver3;
  return c;
}

std::vector<DeltaArm::ArmMechConfig> limbs(DeltaArm::ArmMechConfig (*make)(int))
{
  return {make(0), make(1), make(2)};
}

}  // namespace

TEST(DeltaArm, MotorBookkeeping)
{
  DeltaArm::Motor m(30.0f);
  EXPECT_FLOAT_EQ(m.start_pos, 30.0f);
  EXPECT_FLOAT_EQ(m.cur_pos, 30.0f);

  m.set_tar_pos(45.0f);
  EXPECT_FLOAT_EQ(m.tar_pos, 45.0f);
}

TEST(DeltaArm, SetTarPosRunsPipeline)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();
  const DeltaArm::TargetResult r = arm.set_tar_pos(0.0f, 0.0f, -0.28f);

  float targets[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(targets);

  ASSERT_EQ(sizeof(targets) / sizeof(targets[0]), 3u);
  EXPECT_TRUE(r.reached) << r.reason;
}

TEST(DeltaArm, EmergencyStopZeroesOutputs)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();
  arm.set_tar_pos(0.0f, 0.0f, -0.28f);

  float targets[3] = {9.0f, 9.0f, 9.0f};
  arm.get_motor_targets(targets);

  // Emergency stop must zero every motor target and the current pose.
  arm.stop();

  arm.get_motor_targets(targets);
  EXPECT_FLOAT_EQ(targets[0], 0.0f);
  EXPECT_FLOAT_EQ(targets[1], 0.0f);
  EXPECT_FLOAT_EQ(targets[2], 0.0f);

  const DeltaArm::Vec3 cur = arm.get_cur_pos();
  EXPECT_FLOAT_EQ(cur.x, 0.0f);
  EXPECT_FLOAT_EQ(cur.y, 0.0f);
  EXPECT_FLOAT_EQ(cur.z, 0.0f);
}

// The IK and FK must be mutually consistent (perlimb closed chain, effector
// triangle apex-down, shoulder pivots on the base-plate corner circle): solving
// IK for a target and then running FK on those motor angles must recover the
// target. Every accepted target must also be well clear of the band edges, so a
// pose that merely grazes the envelope cannot pass by being clamped.
TEST(DeltaArm, IkFkRoundTrip)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  const float kTol = 3.0e-3f;  // 3 mm
  const DeltaArm::Vec3 targets[] = {
      {0.0f, 0.0f, -0.25f},
      {0.0f, 0.0f, -0.28f},
      {0.0f, 0.0f, -0.30f},
      {0.03f, 0.0f, -0.26f},
      {0.0f, 0.03f, -0.26f},
      {-0.02f, 0.04f, -0.28f},
  };

  for (const auto& t : targets) {
    const DeltaArm::TargetResult r = arm.set_tar_pos(t.x, t.y, t.z);
    ASSERT_TRUE(r.reached) << "(" << t.x << ", " << t.y << ", " << t.z << "): " << r.reason;
    arm.apply();  // promote commanded targets and refresh the FK estimate
    const DeltaArm::Vec3 back = arm.get_cur_pos();
    EXPECT_NEAR(back.x, t.x, kTol) << "target (" << t.x << ", " << t.y << ", " << t.z << ")";
    EXPECT_NEAR(back.y, t.y, kTol) << "target (" << t.x << ", " << t.y << ", " << t.z << ")";
    EXPECT_NEAR(back.z, t.z, kTol) << "target (" << t.x << ", " << t.y << ", " << t.z << ")";
  }
}

// The regression test for the actual bug: sweep the task-space workspace rather
// than a few on-axis points. Any limb whose arm angle leaves [0, 90] deg used to
// be clamped by the forward kinematics, so a large fraction of accepted goals
// streamed a position tens of millimetres from the commanded target. This also
// covers the gen0 assembly, whose band extends below 0 deg.
TEST(DeltaArm, IkFkRoundTripAcrossWorkspace)
{
  const float kTol = 3.0e-3f;  // 3 mm

  for (int variant = 0; variant < 2; ++variant) {
    DeltaArm::Arm arm(nullptr);
    arm.init();
    arm.set_geometry(limbs(variant == 0 ? &defaultLimb : &gen0Limb));
    const char * name = (variant == 0) ? "default" : "gen0";

    int accepted = 0;
    int checked = 0;
    for (int zi = -24; zi >= -35; --zi) {
      for (int xi = -12; xi <= 12; xi += 2) {
        for (int yi = -12; yi <= 12; yi += 2) {
          const float tx = xi * 0.01f;
          const float ty = yi * 0.01f;
          const float tz = zi * 0.01f;

          const DeltaArm::TargetResult r = arm.set_tar_pos(tx, ty, tz);
          if (!r.reached) {
            // A rejection is fine, but it must explain itself.
            EXPECT_FALSE(r.reason.empty()) << name;
            continue;
          }
          ++accepted;

          arm.apply();
          const DeltaArm::Vec3 back = arm.get_cur_pos();
          ++checked;
          EXPECT_NEAR(back.x, tx, kTol) << name << " target (" << tx << ", " << ty << ", " << tz << ")";
          EXPECT_NEAR(back.y, ty, kTol) << name << " target (" << tx << ", " << ty << ", " << tz << ")";
          EXPECT_NEAR(back.z, tz, kTol) << name << " target (" << tx << ", " << ty << ", " << tz << ")";
        }
      }
    }

    // Sanity: the sweep must actually have exercised the machine, otherwise the
    // loop bounds above are wrong and the test is vacuous.
    EXPECT_GT(accepted, 100) << name << ": only " << accepted << " reachable targets found";
    EXPECT_EQ(checked, accepted) << name;
  }
}

// A goal that needs a limb arm angle above 90 deg must be reached EXACTLY. This
// is the core regression: the forward kinematics used to clamp any arm angle to
// 90 deg, so ~37% of all accepted default goals streamed a position far from
// the commanded target while the motor angles were correct. The 145 deg servo
// stop puts the envelope's upper edge at ~99.7 deg, so there is a real band of
// reachable poses that need 90..99.7 deg.
TEST(DeltaArm, IkFkRoundTripAboveNinetyDegrees)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  const DeltaArm::Vec3 t{-0.12f, -0.07f, -0.25f};
  const DeltaArm::TargetResult r = arm.set_tar_pos(t.x, t.y, t.z);
  ASSERT_TRUE(r.reached) << r.reason;

  float motors[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(motors);
  const float arms[3] = {arm.arm_angle_from_motor_deg(motors[0], 0),
                         arm.arm_angle_from_motor_deg(motors[1], 1),
                         arm.arm_angle_from_motor_deg(motors[2], 2)};

  // Prove the target really does need an arm past 90 deg, so this test would
  // have failed before the fix rather than passing by accident.
  float max_arm = 0.0f;
  for (int i = 0; i < 3; ++i) max_arm = std::fmax(max_arm, arms[i]);
  EXPECT_GT(max_arm, 90.0f) << "expected a limb above 90 deg, got " << max_arm;

  arm.apply();
  const DeltaArm::Vec3 back = arm.get_cur_pos();
  EXPECT_NEAR(back.x, t.x, 3.0e-3f);
  EXPECT_NEAR(back.y, t.y, 3.0e-3f);
  EXPECT_NEAR(back.z, t.z, 3.0e-3f);
}

// The whole 90..99.7 deg slice of the envelope, not just one sample. Before the
// fix every one of these streamed a position tens of mm from its target.
TEST(DeltaArm, EveryAcceptedGoalNeedingOverNinetyDegreesRoundTrips)
{
  DeltaArm::Arm arm(nullptr);
  arm.set_geometry(limbs(&defaultLimb));
  float lo = 0.0f;
  float hi = 0.0f;
  ASSERT_TRUE(arm.get_linkage_band(0, lo, hi));
  ASSERT_GT(hi, 90.0f) << "the >90 deg slice must be non-empty for this test to mean anything";

  const float kTol = 3.0e-3f;
  int over_90 = 0;
  for (int zi = -24; zi >= -32; --zi) {
    for (int xi = -14; xi <= 14; ++xi) {
      for (int yi = -14; yi <= 14; ++yi) {
        const float tx = xi * 0.01f;
        const float ty = yi * 0.01f;
        const float tz = zi * 0.01f;

        const DeltaArm::TargetResult r = arm.set_tar_pos(tx, ty, tz);
        if (!r.reached) continue;

        float motors[3] = {0.0f, 0.0f, 0.0f};
        arm.get_motor_targets(motors);
        float max_arm = 0.0f;
        for (int i = 0; i < 3; ++i)
          max_arm = std::fmax(max_arm, arm.arm_angle_from_motor_deg(motors[i], i));
        if (max_arm <= 90.0f) continue;
        ++over_90;

        arm.apply();
        const DeltaArm::Vec3 back = arm.get_cur_pos();
        EXPECT_NEAR(back.x, tx, kTol) << "target (" << tx << ", " << ty << ", " << tz << ")";
        EXPECT_NEAR(back.y, ty, kTol) << "target (" << tx << ", " << ty << ", " << tz << ")";
        EXPECT_NEAR(back.z, tz, kTol) << "target (" << tx << ", " << ty << ", " << tz << ")";
      }
    }
  }
  EXPECT_GT(over_90, 100) << "only " << over_90 << " goals needed >90 deg; sweep is too narrow";
}

// A pose needing MORE arm angle than the servo can deliver is genuinely
// unreachable, and must now be refused rather than silently clamped. This was
// the worst case in the old build: this exact target was 63.7 mm from the
// command, because leg 1 wanted 120.8 deg and the forward kinematics quietly
// clamped it to 90 deg. The motor angles the old code produced were plausible,
// so nothing looked wrong on screen.
TEST(DeltaArm, PoseNeedingPastServoStopIsRefusedNotClamped)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  const DeltaArm::TargetResult r = arm.set_tar_pos(-0.12f, -0.10f, -0.28f);
  EXPECT_FALSE(r.reached) << "needs a 120.8 deg arm angle, beyond the 145 deg servo stop";
  EXPECT_FALSE(r.reason.empty()) << "the refusal must say which limb and what it needed";

  // Nothing may be committed, so the arm stays put instead of drifting to a
  // pose the operator never asked for.
  float motors[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(motors);
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(motors[i], 0.0f);
    EXPECT_LE(motors[i], 145.0f) << "leg " << i << " must never be commanded past the servo stop";
  }
}

// The 4-bar servo linkage cannot close for a nearly-flat limb, and the servo's
// travel bounds the arm at the other end. A pose outside the envelope must be
// REPORTED (never silently dropped, which is indistinguishable from a dead
// motor) and must leave the previous targets intact.
TEST(DeltaArm, UnreachableGoalIsReportedAndKeepsTargets)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  const DeltaArm::TargetResult ok = arm.set_tar_pos(0.0f, 0.0f, -0.28f);
  ASSERT_TRUE(ok.reached) << ok.reason;
  arm.apply();
  float before[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(before);
  ASSERT_GT(before[0], 0.0f);

  // This pose needs limbs near 0 deg, below the linkage's closing band.
  const DeltaArm::TargetResult bad = arm.set_tar_pos(0.0f, 0.0f, -0.15f);
  EXPECT_FALSE(bad.reached);
  EXPECT_FALSE(bad.reason.empty()) << "a rejection must say why";

  float after[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(after);
  EXPECT_FLOAT_EQ(after[0], before[0]);
  EXPECT_FLOAT_EQ(after[1], before[1]);
  EXPECT_FLOAT_EQ(after[2], before[2]);

  // The commanded task-space pose must not advance either, or the streamed
  // target marker would run away from where the arm actually is.
  const DeltaArm::Vec3 tar = arm.get_tar_pos();
  EXPECT_NEAR(tar.z, -0.28f, 1e-6f);
}

// Through the 4-bar, "lower platform = steeper arm = larger motor angle". For
// two on-axis targets the per-limb motor angles must all sit inside the
// driver's nominal [0,145] deg range and increase with depth.
TEST(DeltaArm, FourBarMotorAnglesInRangeAndMonotone)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  arm.set_tar_pos(0.0f, 0.0f, -0.25f);
  arm.apply();
  float shallow[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(shallow);

  arm.set_tar_pos(0.0f, 0.0f, -0.30f);
  arm.apply();
  float deep[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(deep);

  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(shallow[i], 0.0f);
    EXPECT_LE(deep[i], 145.0f);
    EXPECT_GT(deep[i], shallow[i]) << "limb " << i
                                   << " motor must grow as the platform lowers";
  }
}

// The resolved band must be derived from the geometry and the travel limits,
// not hard-coded. For the shipped assemblies it is ~[32, 100] deg on default
// (top edge set by the 145 deg servo stop) and ~[0, 101] deg on gen0 (top edge
// set by the linkage itself).
TEST(DeltaArm, LinkageBandIsDerivedFromGeometry)
{
  struct Case
  {
    const char * name;
    DeltaArm::ArmMechConfig (*make)(int);
    float expect_lo;
    float expect_hi;
  };
  const Case cases[] = {
      {"default", &defaultLimb, 32.0f, 99.6f},
      {"gen0", &gen0Limb, 0.0f, 100.8f},
  };

  std::vector<float> highs;
  for (const auto & c : cases) {
    DeltaArm::Arm arm(nullptr);
    arm.set_geometry(limbs(c.make));
    float lo = 0.0f;
    float hi = 0.0f;
    ASSERT_TRUE(arm.get_linkage_band(0, lo, hi)) << c.name;
    EXPECT_NEAR(lo, c.expect_lo, 0.5f) << c.name << " band lower edge";
    EXPECT_NEAR(hi, c.expect_hi, 0.5f) << c.name << " band upper edge";
    EXPECT_GT(hi, lo) << c.name;
    highs.push_back(hi);
  }

  // Different linkages must not share a band.
  EXPECT_NE(highs[0], highs[1]) << "default and gen0 resolved to the same upper edge";
}

// Tightening the servo's travel must narrow the arm's envelope: the IK is
// bounded by what the servo can physically hold, not just by what the linkage
// can close.
TEST(DeltaArm, ServoTravelBoundsTheEnvelope)
{
  DeltaArm::Arm wide(nullptr);
  wide.set_geometry(limbs(&defaultLimb));
  float wide_lo = 0.0f;
  float wide_hi = 0.0f;
  ASSERT_TRUE(wide.get_linkage_band(0, wide_lo, wide_hi));

  std::vector<DeltaArm::ArmMechConfig> tight = limbs(&defaultLimb);
  for (auto & c : tight) c.motor_angle_max = 100.0f;
  DeltaArm::Arm narrow(nullptr);
  narrow.set_geometry(tight);
  float narrow_lo = 0.0f;
  float narrow_hi = 0.0f;
  ASSERT_TRUE(narrow.get_linkage_band(0, narrow_lo, narrow_hi));

  EXPECT_NEAR(narrow_lo, wide_lo, 0.5f) << "the linkage (not the servo) sets the lower edge";
  EXPECT_LT(narrow_hi, wide_hi) << "a tighter servo stop must narrow the arm envelope";

  // A deep pose needs a larger motor angle, so it must become unreachable.
  const DeltaArm::TargetResult deep_narrow = narrow.set_tar_pos(0.0f, 0.0f, -0.33f);
  const DeltaArm::TargetResult deep_wide = wide.set_tar_pos(0.0f, 0.0f, -0.33f);
  EXPECT_TRUE(deep_wide.reached) << deep_wide.reason;
  EXPECT_FALSE(deep_narrow.reached);
  EXPECT_FALSE(deep_narrow.reason.empty());
}

// The arm-angle window is a real constraint, not decoration. The gen0 linkage
// folds (its motor angle stops being monotone) right around 0 deg, so opening
// the window below 0 does NOT buy a usable negative-arm envelope - there is
// nothing there to invert. That fold is the real reason the default floor of
// 0 deg costs gen0 nothing, and it is why the default is safe.
TEST(DeltaArm, ArmAngleWindowBoundsTheBandButCannotBeatTheFold)
{
  DeltaArm::Arm floored(nullptr);
  floored.set_geometry(limbs(&gen0Limb));
  float floored_lo = 0.0f;
  float floored_hi = 0.0f;
  ASSERT_TRUE(floored.get_linkage_band(0, floored_lo, floored_hi));
  EXPECT_GE(floored_lo, 0.0f) << "negative arm angles must be off by default";

  // Relaxing the floor must not silently claim a negative envelope: the
  // four-bar's fold is what actually stops it, and the inverse is only valid
  // where the transmission is monotone.
  std::vector<DeltaArm::ArmMechConfig> opened = limbs(&gen0Limb);
  for (auto & c : opened) c.arm_angle_min = -30.0f * kDegToRad;
  DeltaArm::Arm open(nullptr);
  open.set_geometry(opened);
  float open_lo = 0.0f;
  float open_hi = 0.0f;
  ASSERT_TRUE(open.get_linkage_band(0, open_lo, open_hi));
  EXPECT_GE(open_lo, -0.5f) << "gen0's linkage folds at ~0 deg, so no negative band exists";
  EXPECT_NEAR(open_hi, floored_hi, 0.5f) << "the upper edge is set by the linkage, not the window";

  // Raising the floor, on the other hand, must narrow the band from below.
  std::vector<DeltaArm::ArmMechConfig> raised = limbs(&gen0Limb);
  for (auto & c : raised) c.arm_angle_min = 10.0f * kDegToRad;
  DeltaArm::Arm tight(nullptr);
  tight.set_geometry(raised);
  float tight_lo = 0.0f;
  float tight_hi = 0.0f;
  ASSERT_TRUE(tight.get_linkage_band(0, tight_lo, tight_hi));
  EXPECT_NEAR(tight_lo, 10.0f, 0.5f) << "the window must be applied to the resolved band";
  EXPECT_NEAR(tight_hi, floored_hi, 0.5f) << "raising the floor must not move the top edge";

  // And a pose that needs a shallower limb than the raised floor allows is
  // then refused (on-axis, this pose needs ~5.6 deg of arm angle).
  const DeltaArm::TargetResult shallow = tight.set_tar_pos(0.0f, 0.0f, -0.18f);
  EXPECT_FALSE(shallow.reached);
  EXPECT_FALSE(shallow.reason.empty());
}

// The forward solve runs on every control tick and must be total: any motor
// angle, including one at a servo hard stop or from feedback the current
// geometry cannot represent, maps to a finite arm angle inside the band.
TEST(DeltaArm, ArmFromMotorNeverThrowsOverFullRange)
{
  for (int variant = 0; variant < 2; ++variant) {
    DeltaArm::Arm arm(nullptr);
    arm.set_geometry(limbs(variant == 0 ? &defaultLimb : &gen0Limb));
    const char * name = (variant == 0) ? "default" : "gen0";

    float lo = 0.0f;
    float hi = 0.0f;
    ASSERT_TRUE(arm.get_linkage_band(0, lo, hi)) << name;

    for (int deg = -360; deg <= 360; ++deg) {
      const float a = arm.arm_angle_from_motor_deg(static_cast<float>(deg), 0);
      EXPECT_TRUE(std::isfinite(a)) << name << " motor " << deg << " -> " << a;
      EXPECT_GE(a, lo - 1e-3f) << name << " motor " << deg << " below the band";
      EXPECT_LE(a, hi + 1e-3f) << name << " motor " << deg << " above the band";
    }
  }
}

// The inverse is a bisection, so it is only valid where motor_from_arm() is
// monotone. Sweeping the motor angle must produce a monotone arm angle across
// the band, with no fold in the middle.
TEST(DeltaArm, ArmFromMotorIsMonotoneAcrossBand)
{
  for (int variant = 0; variant < 2; ++variant) {
    DeltaArm::Arm arm(nullptr);
    arm.set_geometry(limbs(variant == 0 ? &defaultLimb : &gen0Limb));
    const char * name = (variant == 0) ? "default" : "gen0";

    float lo = 0.0f;
    float hi = 0.0f;
    ASSERT_TRUE(arm.get_linkage_band(0, lo, hi)) << name;

    // Find the motor angles at the band edges by scanning for the clamp
    // points, then check monotonicity strictly between them.
    float prev = -1e9f;
    int inside = 0;
    for (int deg = 0; deg <= 360; ++deg) {
      const float a = arm.arm_angle_from_motor_deg(static_cast<float>(deg), 0);
      if (a <= lo + 1e-3f || a >= hi - 1e-3f) {
        prev = -1e9f;  // outside the band: the clamp is flat, restart
        continue;
      }
      EXPECT_GE(a, prev) << name << ": arm angle fell from " << prev << " to " << a
                         << " at motor " << deg << " deg (four-bar fold inside the band?)";
      prev = a;
      ++inside;
    }
    EXPECT_GT(inside, 10) << name << ": too few in-band samples to be meaningful";
  }
}

// The renderer must be able to tell "the linkage is at this arm angle" apart
// from "the linkage cannot reach this motor angle". The clamping conversion
// hides that: every motor angle past the closure edge collapses onto the same
// band edge, which used to freeze the drawn arm across most of the real servo
// travel while the readout kept moving.
TEST(DeltaArm, CheckedInverseReportsOutOfClosureInsteadOfClamping)
{
  for (int variant = 0; variant < 2; ++variant) {
    DeltaArm::Arm arm(nullptr);
    arm.set_geometry(limbs(variant == 0 ? &defaultLimb : &gen0Limb));
    const char * name = (variant == 0) ? "default" : "gen0";

    float mlo = 0.0f;
    float mhi = 0.0f;
    ASSERT_TRUE(arm.linkage_motor_span(0, mlo, mhi)) << name;
    ASSERT_LT(mlo, mhi) << name << ": closure span must be non-empty";

    // Inside the closure span the checked and clamping conversions must agree
    // exactly - this is the same solve, just without the silent edge case.
    for (float m = mlo + 1.0f; m < mhi; m += 5.0f) {
      float checked = 0.0f;
      ASSERT_TRUE(arm.try_arm_from_motor_deg(m, 0, checked)) << name << " at " << m;
      EXPECT_NEAR(checked, arm.arm_angle_from_motor_deg(m, 0), 1e-2f)
        << name << ": checked and clamping inverse disagree at " << m;
    }

    // Outside it, the checked solve must fail and must NOT write the output.
    float sentinel = -12345.0f;
    EXPECT_FALSE(arm.try_arm_from_motor_deg(mhi + 10.0f, 0, sentinel))
      << name << ": expected out-of-closure above the span";
    EXPECT_FLOAT_EQ(sentinel, -12345.0f)
      << name << ": out-of-closure must leave arm_out untouched";

    float sentinel2 = -12345.0f;
    EXPECT_FALSE(arm.try_arm_from_motor_deg(mlo - 10.0f, 0, sentinel2))
      << name << ": expected out-of-closure below the span";
    EXPECT_FLOAT_EQ(sentinel2, -12345.0f)
      << name << ": out-of-closure must leave arm_out untouched";

    // The whole point: readings that used to collapse onto one clamped angle
    // must now be distinguishable.
    const float clamped_hi = arm.arm_angle_from_motor_deg(mhi + 5.0f, 0);
    const float clamped_far = arm.arm_angle_from_motor_deg(mhi + 40.0f, 0);
    EXPECT_NEAR(clamped_hi, clamped_far, 1e-2f)
      << name << ": clamping still flattens the range (regression guard)";
    EXPECT_FALSE(arm.try_arm_from_motor_deg(mhi + 5.0f, 0, sentinel))
      << name << ": the flattened readings must be reported as out of closure";
  }
}

// A rejected goal must not half-apply: the IK is per limb, and a pose that is
// reachable on legs 0-1 can be unreachable on leg 2. No limb may be committed
// unless all three solve.
TEST(DeltaArm, RejectedGoalCommitsNothing)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  const DeltaArm::TargetResult ok = arm.set_tar_pos(0.0f, 0.0f, -0.28f);
  ASSERT_TRUE(ok.reached) << ok.reason;

  int accepted = 0;
  int refused = 0;
  for (int xi = -20; xi <= 20; xi += 2) {
    for (int yi = -20; yi <= 20; yi += 2) {
      // Snapshot the committed state immediately before each attempt: earlier
      // iterations in this loop legitimately committed their own goals.
      float before[3] = {0.0f, 0.0f, 0.0f};
      arm.get_motor_targets(before);
      const DeltaArm::Vec3 tar_before = arm.get_tar_pos();

      const DeltaArm::TargetResult r = arm.set_tar_pos(xi * 0.01f, yi * 0.01f, -0.30f);
      if (r.reached) {
        ++accepted;
        continue;
      }
      ++refused;

      float now[3] = {0.0f, 0.0f, 0.0f};
      arm.get_motor_targets(now);
      for (int i = 0; i < 3; ++i) {
        EXPECT_FLOAT_EQ(now[i], before[i])
            << "leg " << i << " moved on the rejected goal (" << xi * 0.01f << ", "
            << yi * 0.01f << ", -0.30)";
      }

      const DeltaArm::Vec3 tar_now = arm.get_tar_pos();
      EXPECT_FLOAT_EQ(tar_now.x, tar_before.x) << "commanded pose advanced on a rejected goal";
      EXPECT_FLOAT_EQ(tar_now.y, tar_before.y) << "commanded pose advanced on a rejected goal";
      EXPECT_FLOAT_EQ(tar_now.z, tar_before.z) << "commanded pose advanced on a rejected goal";
    }
  }
  // The sweep must have exercised both outcomes for the check to be meaningful.
  EXPECT_GT(accepted, 10);
  EXPECT_GT(refused, 10);
}

// A geometry whose four-bar can never close within the servo's travel must be
// reported as an empty envelope rather than producing nonsense poses.
TEST(DeltaArm, UnclosableLinkageGivesEmptyBand)
{
  std::vector<DeltaArm::ArmMechConfig> tiny = limbs(&defaultLimb);
  for (auto & c : tiny) {
    c.upper_rod_len = 2.0f;
    c.servo_rod_len = 2.0f;
  }
  DeltaArm::Arm arm(nullptr);
  arm.set_geometry(tiny);

  float lo = 0.0f;
  float hi = 0.0f;
  EXPECT_FALSE(arm.get_linkage_band(0, lo, hi));

  // Every pose is rejected, with a reason.
  const DeltaArm::TargetResult r = arm.set_tar_pos(0.0f, 0.0f, -0.28f);
  EXPECT_FALSE(r.reached);
  EXPECT_FALSE(r.reason.empty());

  // The forward solve still returns something finite.
  arm.apply();
  const DeltaArm::Vec3 cur = arm.get_cur_pos();
  EXPECT_TRUE(std::isfinite(cur.x) && std::isfinite(cur.y) && std::isfinite(cur.z));
}

// set_geometry must validate its input rather than indexing past the end.
TEST(DeltaArm, SetGeometryRejectsWrongLimbCount)
{
  DeltaArm::Arm arm(nullptr);
  EXPECT_THROW(arm.set_geometry({defaultLimb(0)}), std::invalid_argument);
  EXPECT_THROW(arm.set_geometry({defaultLimb(0), defaultLimb(1), defaultLimb(2), defaultLimb(0)}),
               std::invalid_argument);
}
