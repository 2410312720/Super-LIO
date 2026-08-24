#include <gtest/gtest.h>

#include <Eigen/Core>

#include "lio/undistortion_utils.h"

namespace LI2Sup {
namespace {

using Vector3d = Eigen::Vector3d;

TEST(UndistortionUtils, ZeroOffsetReturnsIntervalHeadPosition)
{
  const Vector3d position(1.0, 2.0, 3.0);
  const Vector3d translated = translationAtTimeOffset(
      position, Vector3d(4.0, 5.0, 6.0), Vector3d(7.0, 8.0, 9.0), 0.0);

  EXPECT_TRUE(translated.isApprox(position, 0.0));
}

TEST(UndistortionUtils, PointOffsetUsesConstantAccelerationKinematics)
{
  const Vector3d position(1.0, -2.0, 0.5);
  const Vector3d velocity(2.0, 4.0, -1.0);
  const Vector3d acceleration(0.5, -1.0, 2.0);
  constexpr double tau = 0.003;
  const Vector3d expected =
      position + velocity * tau + 0.5 * acceleration * tau * tau;

  const Vector3d translated =
      translationAtTimeOffset(position, velocity, acceleration, tau);

  EXPECT_TRUE(translated.isApprox(expected, 1e-15));
}

TEST(UndistortionUtils, InteriorPointDoesNotUseFullImuInterval)
{
  const Vector3d position = Vector3d::Zero();
  const Vector3d velocity(1.0, -2.0, 0.5);
  const Vector3d acceleration(0.2, 0.4, -0.8);
  constexpr double imu_interval = 0.005;
  constexpr double point_offset = 0.001;

  const Vector3d point_translation = translationAtTimeOffset(
      position, velocity, acceleration, point_offset);
  const Vector3d interval_end_translation = translationAtTimeOffset(
      position, velocity, acceleration, imu_interval);

  EXPECT_FALSE(point_translation.isApprox(interval_end_translation, 1e-15));
  EXPECT_NEAR(point_translation.x(), 0.0010001, 1e-15);
  EXPECT_NEAR(interval_end_translation.x(), 0.0050025, 1e-15);
}

}  // namespace
}  // namespace LI2Sup
