#include <gtest/gtest.h>

#include <limits>

#include <Eigen/Core>

#include "lio/covariance_utils.h"

namespace LI2Sup {
namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix18d = Eigen::Matrix<double, 18, 18>;

TEST(CovarianceUtils, ObservationInformationOnlyOccupiesObservedStateBlock)
{
  const Matrix6d information = 3.0 * Matrix6d::Identity();
  const Matrix18d embedded = embedObservedInformation<double, 18, 6>(information);
  const Matrix6d observed_block = embedded.block<6, 6>(0, 0);
  const double unobserved_block_norm = embedded.block<12, 12>(6, 6).norm();
  const double upper_cross_block_norm = embedded.block<6, 12>(0, 6).norm();
  const double lower_cross_block_norm = embedded.block<12, 6>(6, 0).norm();

  EXPECT_TRUE(observed_block.isApprox(information, 0.0));
  EXPECT_DOUBLE_EQ(unobserved_block_norm, 0.0);
  EXPECT_DOUBLE_EQ(upper_cross_block_norm, 0.0);
  EXPECT_DOUBLE_EQ(lower_cross_block_norm, 0.0);
}

TEST(CovarianceUtils, InformationPosteriorDoesNotShrinkIndependentUnobservedStates)
{
  Matrix18d prior = Matrix18d::Zero();
  prior.diagonal().head<6>().setConstant(0.5);
  prior.diagonal().tail<12>().setConstant(2.0);
  const Matrix6d information = 4.0 * Matrix6d::Identity();
  const Matrix18d posterior = (
      prior.inverse() + embedObservedInformation<double, 18, 6>(information)).inverse();

  EXPECT_TRUE(posterior.diagonal().head<6>().isApprox(
      Eigen::Matrix<double, 6, 1>::Constant(1.0 / 6.0), 1e-12));
  EXPECT_TRUE(posterior.diagonal().tail<12>().isApprox(
      Eigen::Matrix<double, 12, 1>::Constant(2.0), 1e-12));
}

TEST(CovarianceUtils, StabilizationSymmetrizesSmallNumericalError)
{
  Matrix18d covariance = Matrix18d::Identity();
  covariance(0, 1) = 1e-10;
  covariance(1, 0) = -1e-10;
  Matrix18d stabilized;
  CovarianceValidation<Matrix18d> validation;

  ASSERT_TRUE(stabilizeCovariance(covariance, stabilized, &validation));
  EXPECT_TRUE(validation.finite);
  EXPECT_TRUE(validation.eigensolver_succeeded);
  EXPECT_TRUE(validation.positive_semidefinite);
  EXPECT_NEAR(validation.symmetry_error, 2e-10, 1e-20);
  EXPECT_DOUBLE_EQ((stabilized - stabilized.transpose()).norm(), 0.0);
}

TEST(CovarianceUtils, StabilizationRejectsNonFiniteMatrix)
{
  Matrix18d covariance = Matrix18d::Identity();
  covariance(4, 4) = std::numeric_limits<double>::quiet_NaN();
  Matrix18d stabilized;
  CovarianceValidation<Matrix18d> validation;

  EXPECT_FALSE(stabilizeCovariance(covariance, stabilized, &validation));
  EXPECT_FALSE(validation.finite);
}

TEST(CovarianceUtils, StabilizationRejectsMateriallyIndefiniteMatrix)
{
  Matrix18d covariance = Matrix18d::Identity();
  covariance(7, 7) = -0.1;
  Matrix18d stabilized;
  CovarianceValidation<Matrix18d> validation;

  EXPECT_FALSE(stabilizeCovariance(covariance, stabilized, &validation));
  EXPECT_TRUE(validation.finite);
  EXPECT_TRUE(validation.eigensolver_succeeded);
  EXPECT_FALSE(validation.positive_semidefinite);
  EXPECT_NEAR(validation.minimum_eigenvalue, -0.1, 1e-12);
}

}  // namespace
}  // namespace LI2Sup
