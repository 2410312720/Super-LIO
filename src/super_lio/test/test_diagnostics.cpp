#include <gtest/gtest.h>

#include <vector>

#include "common/diagnostics.h"

namespace LI2Sup {
namespace {

TEST(Diagnostics, EmptyResidualSetIsFiniteAndZero)
{
  const ResidualStatistics result = summarizeResiduals({});
  EXPECT_EQ(result.count, 0U);
  EXPECT_DOUBLE_EQ(result.mean, 0.0);
  EXPECT_DOUBLE_EQ(result.rmse, 0.0);
  EXPECT_DOUBLE_EQ(result.p95, 0.0);
  EXPECT_DOUBLE_EQ(result.maximum, 0.0);
}

TEST(Diagnostics, ResidualStatisticsUseAbsoluteTailAndSignedMean)
{
  const ResidualStatistics result = summarizeResiduals({-0.02, 0.01, 0.03, -0.04});
  EXPECT_EQ(result.count, 4U);
  EXPECT_NEAR(result.mean, -0.005, 1e-12);
  EXPECT_NEAR(result.absolute_mean, 0.025, 1e-12);
  EXPECT_NEAR(result.rmse, std::sqrt(0.00075), 1e-12);
  EXPECT_DOUBLE_EQ(result.p95, 0.04);
  EXPECT_DOUBLE_EQ(result.maximum, 0.04);
}

TEST(Diagnostics, ResidualP95UsesNearestRank)
{
  std::vector<double> residuals;
  for (int value = 1; value <= 20; ++value) {
    residuals.push_back(static_cast<double>(value));
  }
  const ResidualStatistics result = summarizeResiduals(residuals);
  EXPECT_DOUBLE_EQ(result.p95, 19.0);
}

}  // namespace
}  // namespace LI2Sup
