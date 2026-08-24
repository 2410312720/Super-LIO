#ifndef LI2SUP_COMMON_DIAGNOSTICS_H_
#define LI2SUP_COMMON_DIAGNOSTICS_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace LI2Sup {

struct ResidualStatistics {
  std::size_t count = 0;
  double mean = 0.0;
  double absolute_mean = 0.0;
  double rmse = 0.0;
  double p95 = 0.0;
  double maximum = 0.0;
};

inline ResidualStatistics summarizeResiduals(const std::vector<double>& residuals)
{
  ResidualStatistics result;
  result.count = residuals.size();
  if (residuals.empty()) {
    return result;
  }

  std::vector<double> absolute_residuals;
  absolute_residuals.reserve(residuals.size());
  double signed_sum = 0.0;
  double absolute_sum = 0.0;
  double squared_sum = 0.0;
  for (const double residual : residuals) {
    const double absolute = std::abs(residual);
    signed_sum += residual;
    absolute_sum += absolute;
    squared_sum += residual * residual;
    absolute_residuals.push_back(absolute);
  }
  std::sort(absolute_residuals.begin(), absolute_residuals.end());
  const std::size_t p95_index = std::min(
      absolute_residuals.size() - 1,
      static_cast<std::size_t>(std::ceil(0.95 * absolute_residuals.size())) - 1);

  result.mean = signed_sum / static_cast<double>(residuals.size());
  result.absolute_mean = absolute_sum / static_cast<double>(residuals.size());
  result.rmse = std::sqrt(squared_sum / static_cast<double>(residuals.size()));
  result.p95 = absolute_residuals[p95_index];
  result.maximum = absolute_residuals.back();
  return result;
}

struct ScanDiagnosticsData {
  std::uint64_t scan_sequence = 0;
  double scan_start_time = 0.0;
  double scan_end_time = 0.0;

  std::size_t input_point_count = 0;
  std::size_t filtered_point_count = 0;
  std::size_t downsampled_point_count = 0;
  std::size_t knn_candidate_count = 0;
  std::size_t matched_point_count = 0;
  double match_fraction = 0.0;

  ResidualStatistics residual;
  std::vector<double> iteration_rotation_correction_rad;
  std::vector<double> iteration_position_correction_m;
  double scan_rotation_correction_rad = 0.0;
  double scan_position_correction_m = 0.0;

  std::array<double, 6> information_eigenvalues{};
  double information_condition_number = 0.0;
  std::uint32_t information_rank = 0;
  std::array<double, 6> information_weakest_direction{};

  std::array<double, 18> covariance_before_diagonal{};
  std::array<double, 18> covariance_after_diagonal{};
  double covariance_before_min_eigenvalue = 0.0;
  double covariance_after_min_eigenvalue = 0.0;
  double covariance_before_symmetry_error = 0.0;
  double covariance_after_symmetry_error = 0.0;

  double imu_state_end_lag_ms = 0.0;
  std::size_t points_after_last_imu = 0;
  double points_after_last_imu_fraction = 0.0;

  double undistort_time_ms = 0.0;
  double downsample_time_ms = 0.0;
  double observe_time_ms = 0.0;
  double map_update_time_ms = 0.0;
  double output_time_ms = 0.0;
  double total_time_ms = 0.0;
};

}  // namespace LI2Sup

#endif  // LI2SUP_COMMON_DIAGNOSTICS_H_
