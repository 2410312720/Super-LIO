#include "lio/dynamic_filter.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace LI2Sup {

namespace {

constexpr double kMinDynamicResidualMultiplier = 1.000001;

bool sanitizePositive(double& value, const double fallback) {
  if (std::isfinite(value) && value > 0.0) return true;
  value = fallback;
  return false;
}

bool sanitizeNonNegative(double& value, const double fallback) {
  if (std::isfinite(value) && value >= 0.0) return true;
  value = fallback;
  return false;
}

bool sanitizeAtLeastOne(int& value) {
  if (value >= 1) return true;
  value = 1;
  return false;
}

bool sanitizeNonNegative(int& value) {
  if (value >= 0) return true;
  value = 0;
  return false;
}

bool sanitizeWeight(double& value, const double fallback) {
  bool valid = std::isfinite(value) && value >= 0.0 && value <= 1.0;
  if (!std::isfinite(value)) value = fallback;
  value = std::clamp(value, 0.0, 1.0);
  return valid;
}

}  // namespace

bool DynamicFilterOptions::sanitize() {
  bool valid = true;
  valid &= sanitizePositive(temporal_voxel_size, 0.25);
  valid &= sanitizeAtLeastOne(static_confirm_frames);
  valid &= sanitizeAtLeastOne(max_unseen_frames);
  valid &= sanitizeNonNegative(max_static_variance, 0.01);

  valid &= sanitizeNonNegative(residual_base, 0.10);
  valid &= sanitizeNonNegative(residual_range_scale, 0.003);
  if (!std::isfinite(dynamic_residual_multiplier) ||
      dynamic_residual_multiplier <= 1.0) {
    dynamic_residual_multiplier = kMinDynamicResidualMultiplier;
    valid = false;
  }

  valid &= sanitizePositive(robust_scale, 0.15);
  valid &= sanitizePositive(observation_inv_variance, 1000.0);

  valid &= sanitizeWeight(static_weight, 1.0);
  valid &= sanitizeWeight(unknown_weight, 0.25);
  valid &= sanitizeWeight(candidate_weight, 0.05);
  valid &= sanitizeWeight(dynamic_weight, 0.0);

  valid &= sanitizeNonNegative(min_effective_points);
  valid &= sanitizeAtLeastOne(init_collect_frames);
  valid &= sanitizeAtLeastOne(init_static_min_hits);
  valid &= sanitizeNonNegative(init_min_static_points);
  return valid;
}

DynamicPointFilter::DynamicPointFilter(DynamicFilterOptions options) {
  setOptions(options);
}

bool DynamicPointFilter::setOptions(DynamicFilterOptions options) {
  const bool valid = options.sanitize();
  options_ = options;
  return valid;
}

MotionLabel DynamicPointFilter::classify(
    const bool plane_valid, const double residual, const double range) const {
  if (!plane_valid || !std::isfinite(residual) || !std::isfinite(range) ||
      range < 0.0) {
    return MotionLabel::UNKNOWN;
  }

  const double threshold =
      options_.residual_base + options_.residual_range_scale * range;
  if (!std::isfinite(threshold) || threshold < 0.0) {
    return MotionLabel::UNKNOWN;
  }

  const double absolute_residual = std::abs(residual);
  if (absolute_residual <= threshold) {
    return MotionLabel::STATIC_CONFIRMED;
  }
  if (absolute_residual >=
      options_.dynamic_residual_multiplier * threshold) {
    return MotionLabel::DYNAMIC_CANDIDATE;
  }
  return MotionLabel::UNKNOWN;
}

double DynamicPointFilter::labelWeight(const MotionLabel label) const {
  switch (label) {
    case MotionLabel::STATIC_CONFIRMED:
      return options_.static_weight;
    case MotionLabel::DYNAMIC_CANDIDATE:
      return options_.candidate_weight;
    case MotionLabel::DYNAMIC_CONFIRMED:
      return options_.dynamic_weight;
    case MotionLabel::UNKNOWN:
    default:
      return options_.unknown_weight;
  }
}

double DynamicPointFilter::cauchyWeight(const double residual) const {
  if (!std::isfinite(residual) || !std::isfinite(options_.robust_scale) ||
      options_.robust_scale <= 0.0) {
    return 0.0;
  }
  const double ratio = residual / options_.robust_scale;
  if (!std::isfinite(ratio)) return 0.0;
  const double squared_ratio = ratio * ratio;
  if (!std::isfinite(squared_ratio)) return 0.0;
  return 1.0 / (1.0 + squared_ratio);
}

TemporalVoxelMap::TemporalVoxelMap(DynamicFilterOptions options) {
  setOptions(options);
}

bool TemporalVoxelMap::setOptions(DynamicFilterOptions options) {
  const bool valid = options.sanitize();
  const bool voxel_size_changed =
      options_.temporal_voxel_size != options.temporal_voxel_size;
  options_ = options;
  if (voxel_size_changed) clear();
  return valid;
}

std::size_t TemporalVoxelMap::VoxelKeyHash::operator()(
    const VoxelKey& key) const {
  std::size_t seed = std::hash<std::int64_t>{}(key.x);
  seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL +
          (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b97f4a7c15ULL +
          (seed << 6U) + (seed >> 2U);
  return seed;
}

bool TemporalVoxelMap::makeKey(
    const BASIC::V3& point_world, VoxelKey& key) const {
  const double inverse_size = 1.0 / options_.temporal_voxel_size;
  const double min_key = static_cast<double>(
      std::numeric_limits<std::int64_t>::lowest());
  const double max_key = static_cast<double>(
      std::numeric_limits<std::int64_t>::max());
  std::array<double, 3> coordinates{};
  for (int i = 0; i < 3; ++i) {
    coordinates[i] = std::floor(
        static_cast<double>(point_world[i]) * inverse_size);
    if (!std::isfinite(coordinates[i]) || coordinates[i] < min_key ||
        coordinates[i] > max_key) {
      return false;
    }
  }
  key.x = static_cast<std::int64_t>(coordinates[0]);
  key.y = static_cast<std::int64_t>(coordinates[1]);
  key.z = static_cast<std::int64_t>(coordinates[2]);
  return true;
}

BASIC::V3 TemporalVoxelMap::meanPoint(const VoxelState& state) {
  if (state.statistics_count <= 0) return BASIC::V3::Zero();
  const double inverse_count = 1.0 / state.statistics_count;
  return BASIC::V3(
      static_cast<BASIC::scalar>(state.sum[0] * inverse_count),
      static_cast<BASIC::scalar>(state.sum[1] * inverse_count),
      static_cast<BASIC::scalar>(state.sum[2] * inverse_count));
}

double TemporalVoxelMap::positionVariance(const VoxelState& state) {
  if (state.statistics_count <= 0) {
    return std::numeric_limits<double>::infinity();
  }
  const double inverse_count = 1.0 / state.statistics_count;
  const double mean_x = state.sum[0] * inverse_count;
  const double mean_y = state.sum[1] * inverse_count;
  const double mean_z = state.sum[2] * inverse_count;
  const double variance = state.squared_norm_sum * inverse_count -
      (mean_x * mean_x + mean_y * mean_y + mean_z * mean_z);
  return std::max(0.0, variance);
}

TemporalVoxelMap::Points TemporalVoxelMap::update(
    const std::uint64_t frame_id, const Points& points_world) {
  std::unordered_map<VoxelKey, FrameAccumulator, VoxelKeyHash> frame_voxels;
  frame_voxels.reserve(points_world.size());
  for (const auto& point : points_world) {
    VoxelKey key;
    if (!makeKey(point, key)) continue;
    auto& accumulator = frame_voxels[key];
    accumulator.sum[0] += point[0];
    accumulator.sum[1] += point[1];
    accumulator.sum[2] += point[2];
    ++accumulator.count;
  }

  Points promoted_points;
  promoted_points.reserve(frame_voxels.size());
  for (const auto& [key, accumulator] : frame_voxels) {
    if (accumulator.count == 0) continue;
    const double inverse_count = 1.0 / accumulator.count;
    const std::array<double, 3> representative{
        accumulator.sum[0] * inverse_count,
        accumulator.sum[1] * inverse_count,
        accumulator.sum[2] * inverse_count};

    auto [iter, inserted] = voxels_.try_emplace(key);
    VoxelState& state = iter->second;
    if (!inserted && state.last_seen_frame == frame_id) {
      continue;
    }

    const bool consecutive = !inserted && frame_id > 0 &&
        state.last_seen_frame == frame_id - 1;
    if (!consecutive) {
      state.consecutive_frames = 0;
      state.statistics_count = 0;
      state.sum = {0.0, 0.0, 0.0};
      state.squared_norm_sum = 0.0;
    }

    state.last_seen_frame = frame_id;
    ++state.consecutive_frames;
    ++state.statistics_count;
    state.sum[0] += representative[0];
    state.sum[1] += representative[1];
    state.sum[2] += representative[2];
    state.squared_norm_sum +=
        representative[0] * representative[0] +
        representative[1] * representative[1] +
        representative[2] * representative[2];

    if (!state.promoted &&
        state.consecutive_frames >= options_.static_confirm_frames &&
        positionVariance(state) <= options_.max_static_variance) {
      state.promoted = true;
      promoted_points.push_back(meanPoint(state));
    }
  }

  for (auto iter = voxels_.begin(); iter != voxels_.end();) {
    const VoxelState& state = iter->second;
    const bool stale = frame_id > state.last_seen_frame &&
        frame_id - state.last_seen_frame >
            static_cast<std::uint64_t>(options_.max_unseen_frames);
    if (!state.promoted && stale) {
      iter = voxels_.erase(iter);
    } else {
      ++iter;
    }
  }

  return promoted_points;
}

void TemporalVoxelMap::clear() {
  voxels_.clear();
}

int TemporalVoxelMap::consecutiveHits(const BASIC::V3& point_world) const {
  VoxelKey key;
  if (!makeKey(point_world, key)) return 0;
  const auto iter = voxels_.find(key);
  return iter == voxels_.end() ? 0 : iter->second.consecutive_frames;
}

bool TemporalVoxelMap::isPromoted(const BASIC::V3& point_world) const {
  VoxelKey key;
  if (!makeKey(point_world, key)) return false;
  const auto iter = voxels_.find(key);
  return iter != voxels_.end() && iter->second.promoted;
}

}  // namespace LI2Sup
