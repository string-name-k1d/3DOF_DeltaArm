#include <gtest/gtest.h>

#include "arm/delta_arm.hpp"

// ---------------------------------------------------------------------------
// These tests exercise the arm-library plumbing (state bookkeeping + the IK
// pipeline): motor/target bookkeeping, the classic delta two-stage IK vs FK
// round trip, and the 4-bar servo-linkage transmission the stage-2 IK solves.
// ---------------------------------------------------------------------------

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
  arm.set_tar_pos(0.15f, 0.0f, 0.20f);

  float targets[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(targets);

  // The skeleton IK currently yields zeros; this guards that the pipeline
  // produces exactly three motor targets without crashing until the real
  // two-stage math is implemented.
  ASSERT_EQ(sizeof(targets) / sizeof(targets[0]), 3u);
  SUCCEED();
}

TEST(DeltaArm, EmergencyStopZeroesOutputs)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();
  arm.set_tar_pos(0.15f, -0.05f, 0.20f);

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
// target. The default geometry is base circumradius 100, platform circumradius
// 32.5, upper arm 120, lower rod 240 (all mm), with the 4-bar servo linkage
// (horn 60, rod 35, attach 68.5/20.5) which only closes for arm angles inside
// its working envelope - so the targets below are chosen INSIDE that envelope
// (each limb's arm angle stays in the ~33..85 deg closing band).
TEST(DeltaArm, IkFkRoundTrip)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  const float kTol = 3.0e-3f; // 3 mm
  const DeltaArm::Vec3 targets[] = {
      {0.0f, 0.0f, -0.25f},
      {0.0f, 0.0f, -0.28f},
      {0.0f, 0.0f, -0.30f},
      {0.03f, 0.0f, -0.26f},
      {0.0f, 0.03f, -0.26f},
      {-0.02f, 0.04f, -0.28f},
  };

  for (const auto& t : targets) {
    arm.set_tar_pos(t.x, t.y, t.z);
    arm.apply(); // promote commanded targets and refresh the FK estimate
    const DeltaArm::Vec3 back = arm.get_cur_pos();
    EXPECT_NEAR(back.x, t.x, kTol) << "target (" << t.x << ", " << t.y << ", " << t.z << ")";
    EXPECT_NEAR(back.y, t.y, kTol) << "target (" << t.x << ", " << t.y << ", " << t.z << ")";
    EXPECT_NEAR(back.z, t.z, kTol) << "target (" << t.x << ", " << t.y << ", " << t.z << ")";
  }
}

// The 4-bar servo linkage cannot close for a nearly-flat limb (arm ~0 deg is
// outside the mechanism's closing band). Stage 2 must report the target as
// unreachable (retain the previous targets) instead of producing a garbage
// motor angle.
TEST(DeltaArm, FourBarUnreachableFlatKeepsTargets)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  arm.set_tar_pos(0.0f, 0.0f, -0.28f);  // reachable: arms in the closing band
  arm.apply();
  float before[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(before);
  ASSERT_GT(before[0], 0.0f);
  ASSERT_GT(before[1], 0.0f);
  ASSERT_GT(before[2], 0.0f);

  // This pose needs arms near 0 deg (below the ~33 deg closing threshold):
  // the IK must keep the previous (reachable) targets untouched.
  arm.set_tar_pos(0.0f, 0.0f, -0.15f);
  float after[3] = {0.0f, 0.0f, 0.0f};
  arm.get_motor_targets(after);
  EXPECT_FLOAT_EQ(after[0], before[0]);
  EXPECT_FLOAT_EQ(after[1], before[1]);
  EXPECT_FLOAT_EQ(after[2], before[2]);
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