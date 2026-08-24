#include <gtest/gtest.h>

#include "common/ds.h"

namespace LI2Sup {
namespace {

TEST(ImuInterpolation, InterpolatesMeasurementAtScanEnd)
{
  IMUData before;
  before.secs = 10.000;
  before.acc = BASIC::V3(0.0, 2.0, 4.0);
  before.gyr = BASIC::V3(-2.0, 0.0, 2.0);
  IMUData after;
  after.secs = 10.004;
  after.acc = BASIC::V3(4.0, 6.0, 8.0);
  after.gyr = BASIC::V3(2.0, 4.0, 6.0);

  const auto result = interpolateImuAtTime(before, after, 10.001);

  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->secs, 10.001);
  EXPECT_TRUE(result->acc.isApprox(BASIC::V3(1.0, 3.0, 5.0), 1e-12));
  EXPECT_TRUE(result->gyr.isApprox(BASIC::V3(-1.0, 1.0, 3.0), 1e-12));
}

TEST(ImuInterpolation, PreservesExactBracketEndpoints)
{
  IMUData before;
  before.secs = 5.0;
  before.acc = BASIC::V3(1.0, 2.0, 3.0);
  before.gyr = BASIC::V3(4.0, 5.0, 6.0);
  IMUData after;
  after.secs = 5.01;
  after.acc = BASIC::V3(7.0, 8.0, 9.0);
  after.gyr = BASIC::V3(10.0, 11.0, 12.0);

  const auto at_before = interpolateImuAtTime(before, after, before.secs);
  const auto at_after = interpolateImuAtTime(before, after, after.secs);

  ASSERT_TRUE(at_before.has_value());
  ASSERT_TRUE(at_after.has_value());
  EXPECT_TRUE(at_before->acc.isApprox(before.acc, 0.0));
  EXPECT_TRUE(at_before->gyr.isApprox(before.gyr, 0.0));
  EXPECT_TRUE(at_after->acc.isApprox(after.acc, 0.0));
  EXPECT_TRUE(at_after->gyr.isApprox(after.gyr, 0.0));
}

TEST(ImuInterpolation, RejectsMissingOrInvalidBracket)
{
  IMUData before;
  before.secs = 2.0;
  IMUData after;
  after.secs = 2.01;

  EXPECT_FALSE(interpolateImuAtTime(before, after, 1.99).has_value());
  EXPECT_FALSE(interpolateImuAtTime(before, after, 2.02).has_value());
  after.secs = before.secs;
  EXPECT_FALSE(interpolateImuAtTime(before, after, 2.0).has_value());
}

}  // namespace
}  // namespace LI2Sup
