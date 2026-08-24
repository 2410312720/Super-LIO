#ifndef LI2SUP_LIO_COVARIANCE_UTILS_H_
#define LI2SUP_LIO_COVARIANCE_UTILS_H_

#include <algorithm>
#include <limits>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

namespace LI2Sup {

template <typename MatrixType>
struct CovarianceValidation {
  using Scalar = typename MatrixType::Scalar;

  bool finite = false;
  bool eigensolver_succeeded = false;
  bool positive_semidefinite = false;
  Scalar symmetry_error = std::numeric_limits<Scalar>::infinity();
  Scalar minimum_eigenvalue = -std::numeric_limits<Scalar>::infinity();
  Scalar negative_eigenvalue_tolerance = Scalar{0};
};

template <typename Scalar, int StateDimension, int ObservedDimension>
Eigen::Matrix<Scalar, StateDimension, StateDimension> embedObservedInformation(
    const Eigen::Matrix<Scalar, ObservedDimension, ObservedDimension>& information)
{
  Eigen::Matrix<Scalar, StateDimension, StateDimension> embedded =
      Eigen::Matrix<Scalar, StateDimension, StateDimension>::Zero();
  embedded.template block<ObservedDimension, ObservedDimension>(0, 0) = information;
  return embedded;
}

template <typename MatrixType>
bool stabilizeCovariance(
    const MatrixType& candidate,
    MatrixType& stabilized,
    CovarianceValidation<MatrixType>* validation = nullptr)
{
  using Scalar = typename MatrixType::Scalar;
  CovarianceValidation<MatrixType> local_validation;
  auto& result = validation == nullptr ? local_validation : *validation;

  result.finite = candidate.allFinite();
  if (!result.finite) {
    return false;
  }

  result.symmetry_error =
      (candidate - candidate.transpose()).cwiseAbs().maxCoeff();
  const MatrixType symmetric = Scalar{0.5} * (candidate + candidate.transpose());
  Eigen::SelfAdjointEigenSolver<MatrixType> solver(symmetric);
  result.eigensolver_succeeded = solver.info() == Eigen::Success;
  if (!result.eigensolver_succeeded) {
    return false;
  }

  auto eigenvalues = solver.eigenvalues();
  result.minimum_eigenvalue = eigenvalues.minCoeff();
  const Scalar covariance_scale = std::max(
      symmetric.cwiseAbs().maxCoeff(), std::numeric_limits<Scalar>::min());
  result.negative_eigenvalue_tolerance =
      std::numeric_limits<Scalar>::epsilon() * covariance_scale *
      static_cast<Scalar>(MatrixType::RowsAtCompileTime * 32);
  result.positive_semidefinite =
      result.minimum_eigenvalue >= -result.negative_eigenvalue_tolerance;
  if (!result.positive_semidefinite) {
    return false;
  }

  if (result.minimum_eigenvalue < Scalar{0}) {
    const Scalar eigenvalue_floor =
        std::numeric_limits<Scalar>::epsilon() * covariance_scale;
    eigenvalues = eigenvalues.cwiseMax(eigenvalue_floor);
    stabilized = solver.eigenvectors() * eigenvalues.asDiagonal() *
        solver.eigenvectors().transpose();
    stabilized = Scalar{0.5} * (stabilized + stabilized.transpose());
  } else {
    stabilized = symmetric;
  }
  return stabilized.allFinite();
}

}  // namespace LI2Sup

#endif  // LI2SUP_LIO_COVARIANCE_UTILS_H_
