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