#ifndef SUPER_LIO_DYNAMIC_FILTER_H_
#define SUPER_LIO_DYNAMIC_FILTER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>

#include "basic/alias.h"

namespace LI2Sup {

struct DynamicFilterOptions {
  bool enable = false;

  bool filter_pose_update = true;
  bool filter_map_update = true;
  bool filter_saved_map = true;
  bool publish_debug_clouds = true;
  int debug_publish_every_n_frames = 5;

  double temporal_voxel_size = 0.25;
  int static_confirm_frames = 3;
  int max_unseen_frames = 20;
  int max_voxels = 200000;
  double max_static_variance = 0.01;

  double residual_base = 0.10;
  double residual_range_scale = 0.003;
  double dynamic_residual_multiplier = 2.5;

  double static_weight = 1.0;
  double unknown_weight = 0.25;
  double candidate_weight = 0.05;
  double dynamic_weight = 0.0;

  int min_effective_points = 40;
  double min_information_ratio = 0.35;

  int init_collect_frames = 6;
  int init_max_collect_frames = 30;
  int init_static_min_hits = 3;
  int init_min_static_points = 100;

  // Returns true when all values were already valid. Invalid values are
  // replaced with safe defaults or clamped to their documented range.
  bool sanitize();
};

enum class MotionLabel : std::uint8_t {
  UNKNOWN = 0,
  STATIC_CONFIRMED = 1,
  DYNAMIC_CANDIDATE = 2,
  DYNAMIC_CONFIRMED = 3,
};

struct PointMotionEvidence {
  bool knn_valid = false;
  bool plane_valid = false;
  bool residual_inlier = false;
  double residual = std::numeric_limits<double>::quiet_NaN();
  double range = std::numeric_limits<double>::quiet_NaN();
  MotionLabel label = MotionLabel::UNKNOWN;
};

struct DynamicObservationQuality {
  std::size_t effective_point_count = 0;
  double information_ratio = 0.0;
  int baseline_rank = 0;
  int dynamic_rank = 0;
  bool fallback_required = true;
};

// Compares the dynamically weighted observation against the exact baseline
// observation assembled from the same correspondences. Both matrices are
// expected to be symmetric positive semidefinite information matrices.
DynamicObservationQuality evaluateDynamicObservationQuality(
    const BASIC::M6d& dynamic_information,
    const BASIC::M6d& baseline_information,
    std::size_t effective_point_count,
    const DynamicFilterOptions& options);

class DynamicPointFilter {
public:
  explicit DynamicPointFilter(DynamicFilterOptions options = {});

  bool setOptions(DynamicFilterOptions options);
  const DynamicFilterOptions& options() const { return options_; }

  MotionLabel classify(bool plane_valid, double residual, double range) const;
  double labelWeight(MotionLabel label) const;

private:
  DynamicFilterOptions options_;
};

class TemporalVoxelMap {
public:
  using Points = BASIC::VV3;

  explicit TemporalVoxelMap(DynamicFilterOptions options = {});

  bool setOptions(DynamicFilterOptions options);
  const DynamicFilterOptions& options() const { return options_; }

  // Input points must already be in the world frame. One representative per
  // voxel is observed for a frame, regardless of the number of input points.
  Points update(std::uint64_t frame_id, const Points& points_world);

  void clear();
  std::size_t size() const { return voxels_.size(); }
  int consecutiveHits(const BASIC::V3& point_world) const;
  bool isPromoted(const BASIC::V3& point_world) const;

private:
  struct VoxelKey {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const;
  };

  struct FrameAccumulator {
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    std::size_t count = 0;
  };

  struct VoxelState {
    std::uint64_t last_seen_frame = 0;
    int consecutive_frames = 0;
    int statistics_count = 0;
    std::array<double, 3> sum{0.0, 0.0, 0.0};
    double squared_norm_sum = 0.0;
    bool promoted = false;
  };

  bool makeKey(const BASIC::V3& point_world, VoxelKey& key) const;
  static BASIC::V3 meanPoint(const VoxelState& state);
  static double positionVariance(const VoxelState& state);

  DynamicFilterOptions options_;
  std::unordered_map<VoxelKey, VoxelState, VoxelKeyHash> voxels_;
};

}  // namespace LI2Sup

#endif  // SUPER_LIO_DYNAMIC_FILTER_H_
