#include <cmath>

#include <gtest/gtest.h>

#include "lio/initial_pose.h"

namespace LI2Sup {
namespace {

constexpr double kTolerance = 1e-9;

void expectVectorNear(const BASIC::V3& actual, const BASIC::V3& expected) {
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(actual[i], expected[i], kTolerance);
  }
}

void expectMatrixNear(const BASIC::M3& actual, const BASIC::M3& expected) {
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(actual(row, col), expected(row, col), kTolerance);
    }
  }
}

TEST(InitialPoseAlignmentTest, DisabledPreservesLegacyPoseExactly) {
  const BASIC::M3 local_rotation =
      Eigen::AngleAxis<BASIC::scalar>(
          0.3, BASIC::V3::UnitZ()).toRotationMatrix();
  const BASIC::SE3 local_pose(
      local_rotation, BASIC::V3(0.2, -0.1, 0.4));
  const InitialPoseOptions options;

  const BASIC::SE3 result = applyInitialPoseAlignment(local_pose, options);

  expectMatrixNear(result.R_, local_pose.R_);
  expectVectorNear(result.t_, local_pose.t_);
}

TEST(InitialPoseAlignmentTest, OfficeTranslationMovesLocalOrigin) {
  InitialPoseOptions options;
  options.enabled = true;
  options.world_from_lio = BASIC::SE3(
      BASIC::SO3(BASIC::M3::Identity()), BASIC::V3(7.0, 5.35, 0.0));

  const BASIC::SE3 result = applyInitialPoseAlignment(
      BASIC::SE3(), options);

  expectMatrixNear(result.R_, BASIC::M3::Identity());
  expectVectorNear(result.t_, BASIC::V3(7.0, 5.35, 0.0));
}

TEST(InitialPoseAlignmentTest, AppliesFullRigidTransformToLegacyPose) {
  const BASIC::M3 yaw_90 =
      Eigen::AngleAxis<BASIC::scalar>(
          M_PI / 2.0, BASIC::V3::UnitZ()).toRotationMatrix();
  InitialPoseOptions options;
  options.enabled = true;
  options.world_from_lio = BASIC::SE3(
      yaw_90, BASIC::V3(10.0, 20.0, 1.0));
  const BASIC::SE3 local_pose(
      BASIC::SO3(BASIC::M3::Identity()), BASIC::V3(2.0, 3.0, 4.0));

  const BASIC::SE3 result = applyInitialPoseAlignment(local_pose, options);

  expectMatrixNear(result.R_, yaw_90);
  expectVectorNear(result.t_, BASIC::V3(7.0, 22.0, 5.0));
}

}  // namespace
}  // namespace LI2Sup
