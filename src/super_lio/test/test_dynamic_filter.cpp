#include <cmath>
#include <gtest/gtest.h>

#include "lio/dynamic_filter.h"

namespace LI2Sup {
namespace {

DynamicFilterOptions testOptions() {
  DynamicFilterOptions options;
  options.temporal_voxel_size = 1.0;
  options.static_confirm_frames = 3;
  options.max_unseen_frames = 2;
  options.max_static_variance = 0.01;
  options.residual_base = 0.1;
  options.residual_range_scale = 0.0;
  options.dynamic_residual_multiplier = 2.5;
  return options;
}

TEST(DynamicPointFilterTest, InvalidPlaneIsUnknown) {
  DynamicPointFilter filter(testOptions());
  EXPECT_EQ(filter.classify(false, 0.0, 1.0), MotionLabel::UNKNOWN);
}

TEST(DynamicPointFilterTest, SmallResidualIsStatic) {
  DynamicPointFilter filter(testOptions());
  EXPECT_EQ(filter.classify(true, 0.05, 1.0),
            MotionLabel::STATIC_CONFIRMED);
}

TEST(DynamicPointFilterTest, IntermediateResidualIsUnknown) {
  DynamicPointFilter filter(testOptions());
  EXPECT_EQ(filter.classify(true, 0.15, 1.0), MotionLabel::UNKNOWN);
}

TEST(DynamicPointFilterTest, LargeResidualIsDynamicCandidate) {
  DynamicPointFilter filter(testOptions());
  EXPECT_EQ(filter.classify(true, 0.3, 1.0),
            MotionLabel::DYNAMIC_CANDIDATE);
}

TEST(DynamicPointFilterTest, LabelWeightsMatchOptions) {
  DynamicFilterOptions options = testOptions();
  options.static_weight = 0.9;
  options.unknown_weight = 0.4;
  options.candidate_weight = 0.2;
  options.dynamic_weight = 0.1;
  DynamicPointFilter filter(options);
  EXPECT_DOUBLE_EQ(filter.labelWeight(MotionLabel::STATIC_CONFIRMED), 0.9);
  EXPECT_DOUBLE_EQ(filter.labelWeight(MotionLabel::UNKNOWN), 0.4);
  EXPECT_DOUBLE_EQ(filter.labelWeight(MotionLabel::DYNAMIC_CANDIDATE), 0.2);
  EXPECT_DOUBLE_EQ(filter.labelWeight(MotionLabel::DYNAMIC_CONFIRMED), 0.1);
}

TEST(DynamicPointFilterTest, InvalidOptionsAreSanitized) {
  DynamicFilterOptions options;
  options.temporal_voxel_size = 0.0;
  options.dynamic_residual_multiplier = 1.0;
  options.unknown_weight = 2.0;
  options.min_information_ratio = 2.0;
  options.init_collect_frames = 10;
  options.init_max_collect_frames = 5;
  options.init_static_min_hits = 20;
  options.static_confirm_frames = 0;
  EXPECT_FALSE(options.sanitize());
  EXPECT_GT(options.temporal_voxel_size, 0.0);
  EXPECT_GT(options.dynamic_residual_multiplier, 1.0);
  EXPECT_DOUBLE_EQ(options.unknown_weight, 1.0);
  EXPECT_DOUBLE_EQ(options.min_information_ratio, 1.0);
  EXPECT_EQ(options.init_max_collect_frames, options.init_collect_frames);
  EXPECT_EQ(options.init_static_min_hits, options.init_max_collect_frames);
  EXPECT_EQ(options.static_confirm_frames, 1);
}

TEST(DynamicObservationQualityTest, FullInformationDoesNotFallback) {
  DynamicFilterOptions options = testOptions();
  options.min_effective_points = 10;
  options.min_information_ratio = 0.35;
  const BASIC::M6d baseline = BASIC::M6d::Identity() * 100.0;
  const DynamicObservationQuality quality = evaluateDynamicObservationQuality(
      baseline, baseline, 10, options);
  EXPECT_FALSE(quality.fallback_required);
  EXPECT_DOUBLE_EQ(quality.information_ratio, 1.0);
  EXPECT_EQ(quality.baseline_rank, 6);
  EXPECT_EQ(quality.dynamic_rank, 6);
}

TEST(DynamicObservationQualityTest, LowInformationRatioFallsBack) {
  DynamicFilterOptions options = testOptions();
  options.min_effective_points = 10;
  options.min_information_ratio = 0.35;
  const BASIC::M6d baseline = BASIC::M6d::Identity() * 100.0;
  const BASIC::M6d dynamic = baseline * 0.1;
  const DynamicObservationQuality quality = evaluateDynamicObservationQuality(
      dynamic, baseline, 100, options);
  EXPECT_TRUE(quality.fallback_required);
  EXPECT_NEAR(quality.information_ratio, 0.1, 1e-12);
}

TEST(DynamicObservationQualityTest, TooFewEffectivePointsFallBack) {
  DynamicFilterOptions options = testOptions();
  options.min_effective_points = 10;
  options.min_information_ratio = 0.0;
  const BASIC::M6d baseline = BASIC::M6d::Identity();
  const DynamicObservationQuality quality = evaluateDynamicObservationQuality(
      baseline, baseline, 9, options);
  EXPECT_TRUE(quality.fallback_required);
}

TEST(DynamicObservationQualityTest, LostObservableDirectionFallsBack) {
  DynamicFilterOptions options = testOptions();
  options.min_effective_points = 1;
  options.min_information_ratio = 0.0;
  const BASIC::M6d baseline = BASIC::M6d::Identity();
  BASIC::M6d dynamic = baseline;
  dynamic(5, 5) = 0.0;
  const DynamicObservationQuality quality = evaluateDynamicObservationQuality(
      dynamic, baseline, 100, options);
  EXPECT_TRUE(quality.fallback_required);
  EXPECT_EQ(quality.baseline_rank, 6);
  EXPECT_EQ(quality.dynamic_rank, 5);
}

TEST(TemporalVoxelMapTest, MultiplePointsInOneFrameCountOnce) {
  TemporalVoxelMap map(testOptions());
  TemporalVoxelMap::Points points{BASIC::V3(0.1, 0.1, 0.1),
                                  BASIC::V3(0.2, 0.2, 0.2)};
  EXPECT_TRUE(map.update(1, points).empty());
  EXPECT_EQ(map.consecutiveHits(points.front()), 1);
}

TEST(TemporalVoxelMapTest, StableConsecutiveFramesPromote) {
  TemporalVoxelMap map(testOptions());
  TemporalVoxelMap::Points point{BASIC::V3(0.1, 0.1, 0.1)};
  EXPECT_TRUE(map.update(1, point).empty());
  EXPECT_TRUE(map.update(2, point).empty());
  EXPECT_EQ(map.update(3, point).size(), 1U);
  EXPECT_TRUE(map.isPromoted(point.front()));
}

TEST(TemporalVoxelMapTest, VoxelPromotesOnlyOnce) {
  TemporalVoxelMap map(testOptions());
  TemporalVoxelMap::Points point{BASIC::V3(0.1, 0.1, 0.1)};
  map.update(1, point);
  map.update(2, point);
  EXPECT_EQ(map.update(3, point).size(), 1U);
  EXPECT_TRUE(map.update(4, point).empty());
}

TEST(TemporalVoxelMapTest, MissingFrameResetsConsecutiveCount) {
  TemporalVoxelMap map(testOptions());
  TemporalVoxelMap::Points point{BASIC::V3(0.1, 0.1, 0.1)};
  map.update(1, point);
  map.update(3, point);
  EXPECT_EQ(map.consecutiveHits(point.front()), 1);
  EXPECT_TRUE(map.update(4, point).empty());
  EXPECT_EQ(map.update(5, point).size(), 1U);
}

TEST(TemporalVoxelMapTest, HighVarianceDoesNotPromote) {
  DynamicFilterOptions options = testOptions();
  options.temporal_voxel_size = 10.0;
  options.max_static_variance = 0.001;
  TemporalVoxelMap map(options);
  map.update(1, {BASIC::V3(0.1, 0.1, 0.1)});
  map.update(2, {BASIC::V3(0.9, 0.9, 0.9)});
  EXPECT_TRUE(map.update(3, {BASIC::V3(0.2, 0.2, 0.2)}).empty());
}

TEST(TemporalVoxelMapTest, StaleUnpromotedVoxelsAreRemoved) {
  TemporalVoxelMap map(testOptions());
  map.update(1, {BASIC::V3(0.1, 0.1, 0.1)});
  EXPECT_EQ(map.size(), 1U);
  map.update(4, {});
  EXPECT_EQ(map.size(), 0U);
}

TEST(TemporalVoxelMapTest, StalePromotedVoxelsAreRemoved) {
  DynamicFilterOptions options = testOptions();
  options.static_confirm_frames = 1;
  TemporalVoxelMap map(options);
  map.update(1, {BASIC::V3(0.1, 0.1, 0.1)});
  EXPECT_EQ(map.size(), 1U);
  map.update(4, {});
  EXPECT_EQ(map.size(), 0U);
}

TEST(TemporalVoxelMapTest, CapacityEvictsOldestVoxels) {
  DynamicFilterOptions options = testOptions();
  options.max_voxels = 2;
  options.max_unseen_frames = 100;
  TemporalVoxelMap map(options);
  map.update(1, {BASIC::V3(0.1, 0.1, 0.1)});
  map.update(2, {BASIC::V3(1.1, 0.1, 0.1)});
  map.update(3, {BASIC::V3(2.1, 0.1, 0.1)});
  EXPECT_EQ(map.size(), 2U);
  EXPECT_EQ(map.consecutiveHits(BASIC::V3(0.1, 0.1, 0.1)), 0);
}

TEST(TemporalVoxelMapTest, DifferentVoxelsAreTrackedSeparately) {
  TemporalVoxelMap map(testOptions());
  TemporalVoxelMap::Points points{BASIC::V3(0.1, 0.1, 0.1),
                                  BASIC::V3(1.1, 0.1, 0.1)};
  map.update(1, points);
  EXPECT_EQ(map.size(), 2U);
  EXPECT_EQ(map.consecutiveHits(points[0]), 1);
  EXPECT_EQ(map.consecutiveHits(points[1]), 1);
}

}  // namespace
}  // namespace LI2Sup
