#ifndef LI2SUP_LIO_UNDISTORTION_UTILS_H_
#define LI2SUP_LIO_UNDISTORTION_UTILS_H_

namespace LI2Sup {

template <typename VectorType, typename TimeType>
VectorType translationAtTimeOffset(
    const VectorType& position,
    const VectorType& velocity,
    const VectorType& acceleration,
    const TimeType time_offset)
{
  using Scalar = typename VectorType::Scalar;
  const Scalar tau = static_cast<Scalar>(time_offset);
  return position + velocity * tau + Scalar{0.5} * acceleration * tau * tau;
}

}  // namespace LI2Sup

#endif  // LI2SUP_LIO_UNDISTORTION_UTILS_H_
