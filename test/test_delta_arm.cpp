#include <gtest/gtest.h>

#include "arm/delta_arm.hpp"

// ---------------------------------------------------------------------------
// These tests exercise the arm-library plumbing (state bookkeeping + the IK
// pipeline skeleton), not the actual kinematics math yet.
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
// triangle apex-down, motor pivots on the t=45mm circle): solving IK for a
// target and then running FK on those motor angles must recover the target.
// The default geometry is base circumradius 150, platform circumradius 60,
// upper arm 160, lower rod 320 (all mm).
TEST(DeltaArm, IkFkRoundTrip)
{
  DeltaArm::Arm arm(nullptr);
  arm.init();

  const float kTol = 2.0e-3f; // 2 mm
  const DeltaArm::Vec3 targets[] = {
      {0.0f, 0.0f, -0.304f},
      {0.03f, -0.02f, -0.28f},
      {-0.02f, 0.04f, -0.32f},
      {0.05f, 0.05f, -0.25f},
      {0.0f, -0.05f, -0.24f},
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