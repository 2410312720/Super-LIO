#include <gtest/gtest.h>

#include <stdexcept>

#include "lio/observation_utils.h"

namespace LI2Sup {
namespace {

TEST(ObservationUtils, ParsesSupportedKernelNames)
{
  EXPECT_EQ(robustKernelFromString("none"), RobustKernel::None);
  EXPECT_EQ(robustKernelFromString("huber"), RobustKernel::Huber);
  EXPECT_EQ(robustKernelFromString("cauchy"), RobustKernel::Cauchy);
  EXPECT_THROW(robustKernelFromString("tukey"), std::invalid_argument);
}

TEST(ObservationUtils, DisabledKernelPreservesUnitWeight)
{
  EXPECT_DOUBLE_EQ(
      robustObservationWeight(RobustKernel::None, 100.0, 0.03), 1.0);
}

TEST(ObservationUtils, HuberOnlyDownweightsBeyondScale)
{
  EXPECT_DOUBLE_EQ(
      robustObservationWeight(RobustKernel::Huber, 0.02, 0.03), 1.0);
  EXPECT_DOUBLE_EQ(
      robustObservationWeight(RobustKernel::Huber, -0.03, 0.03), 1.0);
  EXPECT_NEAR(
      robustObservationWeight(RobustKernel::Huber, 0.06, 0.03), 0.5, 1e-15);
}

TEST(ObservationUtils, CauchySmoothlyDownweightsResiduals)
{
  EXPECT_DOUBLE_EQ(
      robustObservationWeight(RobustKernel::Cauchy, 0.0, 0.03), 1.0);
  EXPECT_NEAR(
      robustObservationWeight(RobustKernel::Cauchy, 0.03, 0.03), 0.5, 1e-15);
  EXPECT_NEAR(
      robustObservationWeight(RobustKernel::Cauchy, -0.06, 0.03), 0.2, 1e-15);
}

TEST(ObservationUtils, EnabledKernelRejectsInvalidScale)
{
  EXPECT_THROW(
      robustObservationWeight(RobustKernel::Huber, 0.1, 0.0),
      std::invalid_argument);
  EXPECT_THROW(
      robustObservationWeight(RobustKernel::Cauchy, 0.1, -1.0),
      std::invalid_argument);
}

}  // namespace
}  // namespace LI2Sup
