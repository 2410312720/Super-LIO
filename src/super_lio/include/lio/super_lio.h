

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <cstdint>
#include <queue>
#include <vector>
#include <iostream>
#include <cassert>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "ros/ROSWrapper.h"

namespace LI2Sup{

class SuperLIO{
public:
  SuperLIO(){};
  ~SuperLIO(){};

  void setROSWrapper(const ROSWrapper::Ptr& wrapper){
    data_wrapper_ = wrapper;
  }
  virtual void init();
  void process();
  void saveMap();
  void printTimeRecord();

protected:
  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  void Propagation_Undistort();
  void DownSample();
  void Observe();
  virtual void UpdateMap();
  void transformCurrentPointsToWorld();
  void updateMapFromCurrentWorldPoints();
  void publishDynamicDebugClouds();
  virtual void Output();
  void caceData();
  void ProcessCaceMap();

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  StateFn state_fn_;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  
  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  std::uint64_t processing_frame_id_ = 0;
  int init_frame_count_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  BASIC::VV3 newly_promoted_points_world_;
  std::vector<BASIC::VV3> init_frame_points_world_;
  std::vector<std::uint8_t> knn_valid_mask_;
  std::vector<std::uint8_t> plane_valid_mask_;
  std::vector<std::uint8_t> residual_inlier_mask_;
  std::vector<int> effect_knn_idxs_;
  std::vector<std::array<double, 4>> abcd_vec_;
  std::vector<float> point_ranges_;
  std::vector<double> point_residuals_;
  std::vector<MotionLabel> motion_labels_;
  std::vector<PointMotionEvidence> motion_evidence_;
  DynamicPointFilter dynamic_filter_;
  TemporalVoxelMap temporal_voxel_map_;
  bool tracking_degraded_this_frame_ = false;
  std::size_t static_point_count_ = 0;
  std::size_t unknown_point_count_ = 0;
  std::size_t dynamic_candidate_count_ = 0;
  std::size_t dynamic_confirmed_count_ = 0;
  double dynamic_ratio_ = 0.0;
  int pcd_index_ = -1;

  Timer time_record_;
};

} // namespace END.

#endif
