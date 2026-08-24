#ifndef LI2SUP_LIO_OBSERVATION_UTILS_H_
#define LI2SUP_LIO_OBSERVATION_UTILS_H_

#include <cmath>
#include <stdexcept>
#include <string>

namespace LI2Sup {

enum class RobustKernel {
  None,
  Huber,
  Cauchy,
};

inline RobustKernel robustKernelFromString(const std::string& name)
{
  if (name == "none") {
    return RobustKernel::None;
  }
  if (name == "huber") {
    return RobustKernel::Huber;
  }
  if (name == "cauchy") {
    return RobustKernel::Cauchy;
  }
  throw std::invalid_argument(
      "lio.observation.robust_kernel must be none, huber, or cauchy");
}

inline double robustObservationWeight(
    const RobustKernel kernel, const double residual, const double scale)
{
  if (kernel == RobustKernel::None) {
    return 1.0;
  }
  if (!std::isfinite(scale) || scale <= 0.0) {
    throw std::invalid_argument("robust observation scale must be finite and positive");
  }

  const double normalized_residual = std::abs(residual) / scale;
  if (kernel == RobustKernel::Huber) {
    return normalized_residual <= 1.0 ? 1.0 : 1.0 / normalized_residual;
  }
  return 1.0 / (1.0 + normalized_residual * normalized_residual);
}

}  // namespace LI2Sup

#endif  // LI2SUP_LIO_OBSERVATION_UTILS_H_
