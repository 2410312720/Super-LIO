
#include "lio/super_lio.h"
#include "lio/undistortion_utils.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_map>
#include <sys/resource.h>
#include <utility>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <Eigen/Eigenvalues>


using namespace BASIC;

namespace LI2Sup{

namespace {

struct InitializationVoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const InitializationVoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct InitializationVoxelKeyHash {
  std::size_t operator()(const InitializationVoxelKey& key) const {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b97f4a7c15ULL +
            (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct InitializationVoxelAccumulator {
  std::array<double, 3> sum{0.0, 0.0, 0.0};
  double squared_norm_sum = 0.0;
  std::size_t count = 0;
};

bool makeInitializationVoxelKey(
    const V3& point, const double voxel_size,
    InitializationVoxelKey& key) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0) return false;
  const double inverse_size = 1.0 / voxel_size;
  const double min_key = static_cast<double>(
      std::numeric_limits<std::int64_t>::lowest());
  const double max_key = static_cast<double>(
      std::numeric_limits<std::int64_t>::max());
  std::array<double, 3> coordinates{};
  for (int i = 0; i < 3; ++i) {
    coordinates[i] = std::floor(
        static_cast<double>(point[i]) * inverse_size);
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

VV3 aggregateInitializationFrame(
    const VV3& points, const double voxel_size) {
  std::unordered_map<InitializationVoxelKey, InitializationVoxelAccumulator,
                     InitializationVoxelKeyHash> voxels;
  voxels.reserve(points.size());
  for (const auto& point : points) {
    InitializationVoxelKey key;
    if (!makeInitializationVoxelKey(point, voxel_size, key)) continue;
    auto& accumulator = voxels[key];
    accumulator.sum[0] += point[0];
    accumulator.sum[1] += point[1];
    accumulator.sum[2] += point[2];
    ++accumulator.count;
  }

  VV3 representatives;
  representatives.reserve(voxels.size());
  for (const auto& [key, accumulator] : voxels) {
    (void)key;
    if (accumulator.count == 0) continue;
    const double inverse_count = 1.0 / accumulator.count;
    representatives.emplace_back(
        static_cast<scalar>(accumulator.sum[0] * inverse_count),
        static_cast<scalar>(accumulator.sum[1] * inverse_count),
        static_cast<scalar>(accumulator.sum[2] * inverse_count));
  }
  return representatives;
}

double initializationPositionVariance(
    const InitializationVoxelAccumulator& accumulator) {
  if (accumulator.count == 0) {
    return std::numeric_limits<double>::infinity();
  }
  const double inverse_count = 1.0 / accumulator.count;
  const double mean_x = accumulator.sum[0] * inverse_count;
  const double mean_y = accumulator.sum[1] * inverse_count;
  const double mean_z = accumulator.sum[2] * inverse_count;
  return std::max(0.0, accumulator.squared_norm_sum * inverse_count -
      (mean_x * mean_x + mean_y * mean_y + mean_z * mean_z));
}

V3 initializationMean(const InitializationVoxelAccumulator& accumulator) {
  const double inverse_count = 1.0 / accumulator.count;
  return V3(
      static_cast<scalar>(accumulator.sum[0] * inverse_count),
      static_cast<scalar>(accumulator.sum[1] * inverse_count),
      static_cast<scalar>(accumulator.sum[2] * inverse_count));
}

}  // namespace

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (!std::isfinite(n) || n < 1e-6) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  if (!std::all_of(abcd.begin(), abcd.end(),
                   [](const double value) { return std::isfinite(value); })) {
    return false;
  }
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (!std::isfinite(dist) || std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return std::isfinite(length) && std::isfinite(error) &&
      length > 81 * error * error;
}


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }

  dynamic_filter_.setOptions(g_dynamic_filter_options);
  temporal_voxel_map_.setOptions(g_dynamic_filter_options);
  
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  if (map_init()) {
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  while(data_wrapper_->sync_measure(measures_)){
    (this->*state_fn_)();
  }
}


bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 50){
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  // First preserve Super-LIO's original local initialization, then optionally
  // place that complete local frame in an externally defined world frame.
  // This is intentionally independent of relocation's registration guess.
  const SE3 local_initial_pose(SO3(rot), g_odom_robo.t_);
  const SE3 aligned_initial_pose =
      applyInitialPoseAlignment(local_initial_pose, g_initial_pose_options);
  state.R = SO3(aligned_initial_pose.R_);
  state.p = aligned_initial_pose.t_;
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  ++init_frame_count_;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  kf_->SetLastObsTime(measures_.lidar.end_time);

  const bool filter_initial_map = g_dynamic_filter_options.enable &&
      g_dynamic_filter_options.filter_map_update;
  if (!filter_initial_map) {
    ivox_->insert(points_world_v3_);
    // Preserve the original four-frame initialization when filtering is off.
    if(init_frame_count_ > 3){
      g_flg_map_init = false;
      return true;
    }
    return false;
  }

  init_frame_points_world_.push_back(aggregateInitializationFrame(
      points_world_v3_, g_dynamic_filter_options.temporal_voxel_size));
  const std::size_t maximum_initialization_frames = static_cast<std::size_t>(
      g_dynamic_filter_options.init_max_collect_frames);
  if (init_frame_points_world_.size() > maximum_initialization_frames) {
    init_frame_points_world_.erase(init_frame_points_world_.begin());
  }
  if (init_frame_count_ < g_dynamic_filter_options.init_collect_frames) {
    return false;
  }

  std::unordered_map<InitializationVoxelKey, InitializationVoxelAccumulator,
                     InitializationVoxelKeyHash> initialization_voxels;
  for (const auto& frame_points : init_frame_points_world_) {
    for (const auto& point : frame_points) {
      InitializationVoxelKey key;
      if (!makeInitializationVoxelKey(
              point, g_dynamic_filter_options.temporal_voxel_size, key)) {
        continue;
      }
      auto& accumulator = initialization_voxels[key];
      accumulator.sum[0] += point[0];
      accumulator.sum[1] += point[1];
      accumulator.sum[2] += point[2];
      accumulator.squared_norm_sum +=
          static_cast<double>(point[0]) * point[0] +
          static_cast<double>(point[1]) * point[1] +
          static_cast<double>(point[2]) * point[2];
      ++accumulator.count;
    }
  }

  VV3 static_seed_points;
  static_seed_points.reserve(initialization_voxels.size());
  for (const auto& [key, accumulator] : initialization_voxels) {
    (void)key;
    if (accumulator.count == 0) continue;
    const V3 mean = initializationMean(accumulator);
    if (accumulator.count >= static_cast<std::size_t>(
            g_dynamic_filter_options.init_static_min_hits) &&
        initializationPositionVariance(accumulator) <=
            g_dynamic_filter_options.max_static_variance) {
      static_seed_points.push_back(mean);
    }
  }

  if (static_seed_points.empty() ||
      static_seed_points.size() < static_cast<std::size_t>(
          g_dynamic_filter_options.init_min_static_points)) {
    LOG_EVERY_N(WARNING, 10) << YELLOW
                 << " ---> [DynamicFilter] Only " << static_seed_points.size()
                 << " confirmed static initialization voxels; need "
                 << g_dynamic_filter_options.init_min_static_points
                 << ". Continue collecting without publishing odometry."
                 << RESET;
    return false;
  }
  ivox_->insert(static_seed_points);
  init_frame_points_world_.clear();
  init_frame_points_world_.shrink_to_fit();
  temporal_voxel_map_.clear();
  if (!static_seed_points.empty()) {
    g_flg_map_init = false;
    return true;
  }
  LOG(WARNING) << YELLOW
               << " ---> [DynamicFilter] Initialization contained no finite points; collecting another frame."
               << RESET;
  init_frame_count_ = 0;
  return false;
}


void SuperLIO::stateProcess(){
  ++processing_frame_id_;
  const std::uint64_t scan_sequence = diagnostic_scan_sequence_++;
  collect_diagnostics_this_frame_ = g_publish_diagnostics &&
      scan_sequence % static_cast<std::uint64_t>(
          g_diagnostics_publish_every_n_frames) == 0;
  if (collect_diagnostics_this_frame_) {
    scan_diagnostics_ = ScanDiagnosticsData{};
    scan_diagnostics_.scan_sequence = scan_sequence;
    scan_diagnostics_.scan_start_time = measures_.lidar.start_time;
    scan_diagnostics_.scan_end_time = measures_.lidar.end_time;
    scan_diagnostics_.input_point_count = measures_.lidar.input_point_count;

    const auto total_start = std::chrono::steady_clock::now();
    const auto run_stage = [this](auto&& function, const std::string& name, double& elapsed_ms) {
      const auto start = std::chrono::steady_clock::now();
      if (g_time_eva) {
        time_record_.Evaluate(std::forward<decltype(function)>(function), name);
      } else {
        std::forward<decltype(function)>(function)();
      }
      elapsed_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
    };

    run_stage([this]() { Propagation_Undistort(); }, "[Undistort]",
              scan_diagnostics_.undistort_time_ms);
    run_stage([this]() { DownSample(); }, "[DownSample]",
              scan_diagnostics_.downsample_time_ms);
    run_stage([this]() { Observe(); }, "[Observe]",
              scan_diagnostics_.observe_time_ms);
    run_stage([this]() { UpdateMap(); }, "[UpdateMap]",
              scan_diagnostics_.map_update_time_ms);
    run_stage([this]() { Output(); }, "[Output]",
              scan_diagnostics_.output_time_ms);
    scan_diagnostics_.total_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - total_start).count();
    data_wrapper_->pub_scan_diagnostics(scan_diagnostics_);
    caceData();
    return;
  }
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();}, "[Undistort]");
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
  }else{
    Propagation_Undistort();
    DownSample();
    Observe();
    UpdateMap();
  }
  Output();
  caceData();
}


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if (g_dynamic_filter_options.enable &&
      g_dynamic_filter_options.filter_saved_map) {
    world_pc_->clear();
    if (!tracking_degraded_this_frame_ &&
        points_world_v3_.size() == motion_labels_.size() &&
        ds_undistort_->size() == motion_labels_.size()) {
      world_pc_->reserve(static_point_count_ + newly_promoted_points_world_.size());
      for (std::size_t i = 0; i < motion_labels_.size(); ++i) {
        if (motion_labels_[i] != MotionLabel::STATIC_CONFIRMED) continue;
        PointType point = ds_undistort_->points[i];
        point.x = points_world_v3_[i][0];
        point.y = points_world_v3_[i][1];
        point.z = points_world_v3_[i][2];
        world_pc_->push_back(point);
      }
      for (const auto& point_world : newly_promoted_points_world_) {
        PointType point;
        point.x = point_world[0];
        point.y = point_world[1];
        point.z = point_world[2];
        point.intensity = 0.0F;
        world_pc_->push_back(point);
      }
    }
    world_pc_->width = world_pc_->size();
    world_pc_->height = 1;
    world_pc_->is_dense = true;
  } else if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string cmd = "rm -rf " + g_save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + g_save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                               std::string(".pcd"));
    LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
    pcl::io::savePCDFileBinary(map_name, *point_map_);
    point_map_->clear();
    scan_wait_num = 0;
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  PointCloudType::Ptr merged_map(new PointCloudType());

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find("scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string() 
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    
    voxel_filter.setInputCloud(merged_map);
    voxel_filter.filter(filtered_map);
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  if (filtered_map.size() > 0) {
    filtered_map.width = filtered_map.size();
    filtered_map.height = 1;
    filtered_map.is_dense = false;
  }

  pcl::io::savePCDFileBinary(output_map_name, filtered_map);

  LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
  LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      pcl::io::savePCDFileBinary(map_name, *point_map_);
      point_map_->clear();
    }
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
    ProcessCaceMap();
    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    pcl::VoxelGrid<PointType> voxel_fliter;
    PointCloudType latst_map;
    voxel_fliter.setInputCloud(point_map_);
    voxel_fliter.setLeafSize(g_map_ds_size, g_map_ds_size, g_map_ds_size);
    voxel_fliter.filter(latst_map);
    if(latst_map.size() > 0){
      latst_map.width = latst_map.size();
      latst_map.height = 1;
      latst_map.is_dense = false;
    }
    pcl::io::savePCDFileBinary(map_name, latst_map);
    LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
    LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
  }
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }
  if (measures_.imu_at_lidar_end) {
    kf_->Predict(*measures_.imu_at_lidar_end);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;

  if (collect_diagnostics_this_frame_) {
    scan_diagnostics_.filtered_point_count = raw_pc->size();
    const double last_state_time = propagate_states_.back().time;
    scan_diagnostics_.imu_state_end_lag_ms =
        std::max(0.0, measures_.lidar.end_time - last_state_time) * 1000.0;
    scan_diagnostics_.points_after_last_imu = static_cast<std::size_t>(
        std::count_if(raw_pc->points.begin(), raw_pc->points.end(),
                      [start_time, last_state_time](const auto& point) {
                        return start_time + point.offset_time > last_state_time;
                      }));
    scan_diagnostics_.points_after_last_imu_fraction = raw_pc->empty() ? 0.0 :
        static_cast<double>(scan_diagnostics_.points_after_last_imu) /
        static_cast<double>(raw_pc->size());
  }

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;
      if (query_time > propagate_states_.back().time) {
        V3 raw(pt.x, pt.y, pt.z);
        V3 eigen_point = TLI_R * raw + TLI_t;
        pt_full.x = eigen_point[0];
        pt_full.y = eigen_point[1];
        pt_full.z = eigen_point[2];
        continue;
      }
      auto match_iter = propagate_states_.begin();
      for (auto iter = propagate_states_.begin(); iter != propagate_states_.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      auto match_iter_n = std::next(match_iter);
      double dt = match_iter_n->time - match_iter->time;
      double tau = query_time - match_iter->time;
      double s = tau / dt;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      V3 t_ei(
          translationAtTimeOffset(p_h, v_h, acc_t, tau) - T_end_t);
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
  if (collect_diagnostics_this_frame_) {
    scan_diagnostics_.downsampled_point_count = ds_undistort_->size();
  }
}


struct ThreadACC{
  M6d dynamic_HTVH = M6d::Zero();
  V6d dynamic_HTVr = V6d::Zero();
  M6d baseline_HTVH = M6d::Zero();
  V6d baseline_HTVr = V6d::Zero();
  std::size_t dynamic_effective_points = 0;
  std::vector<double> residuals;
};


struct ObservationIterationSummary {
  std::size_t candidate_count = 0;
  M6d information = M6d::Zero();
  ResidualStatistics residual;
};


void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();

  points_body_v3_.resize(ptsize);
  point_ranges_.resize(ptsize);

  knn_valid_mask_.assign(ptsize, 0);
  plane_valid_mask_.assign(ptsize, 0);
  residual_inlier_mask_.assign(ptsize, 0);
  abcd_vec_.assign(ptsize, std::array<double, 4>{});
  point_residuals_.assign(ptsize, std::numeric_limits<double>::quiet_NaN());
  motion_labels_.assign(ptsize, MotionLabel::UNKNOWN);
  motion_evidence_.assign(ptsize, PointMotionEvidence{});
  effect_knn_idxs_.resize(ptsize);
  tracking_degraded_this_frame_ = false;
  dynamic_observation_quality_ = DynamicObservationQuality{};
  dynamic_fallback_required_this_frame_ = false;
  dynamic_fallback_used_this_frame_ = false;
  dynamic_map_update_skipped_this_frame_ = false;

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    point_ranges_[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;
  const SysState state_before = kf_->GetSysState();
  const V3 gravity_before = kf_->GetGravity();
  const ESKF::COV covariance_before = kf_->GetCov();
  std::vector<ObservationIterationSummary> observation_iterations;
  std::vector<ESKF::IterationDiagnostic> filter_iterations;
  ESKF::IterationDiagnosticFunc iteration_diagnostic;
  if (collect_diagnostics_this_frame_) {
    iteration_diagnostic = [&](const ESKF::IterationDiagnostic& diagnostic) {
      filter_iterations.push_back(diagnostic);
    };
  }

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              knn_valid_mask_[idx] = 0;
              plane_valid_mask_[idx] = 0;
              residual_inlier_mask_[idx] = 0;
              motion_evidence_[idx].knn_valid = false;
              motion_evidence_[idx].plane_valid = false;
              continue;
            }
            knn_valid_mask_[idx] = 1;
            plane_valid_mask_[idx] = calc_plane_coeff(
              top_K.count, top_K.points_, abcd_vec_[idx]);
            motion_evidence_[idx].knn_valid = true;
            motion_evidence_[idx].plane_valid = plane_valid_mask_[idx] != 0;
          }

          if(!plane_valid_mask_[idx]) continue;
          if(need_converge && !residual_inlier_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          residual_inlier_mask_[idx] = compute_error(
            abcd, point_world, point_ranges_[idx], error);
          point_residuals_[idx] = static_cast<double>(error);
          motion_labels_[idx] = dynamic_filter_.classify(
            true, point_residuals_[idx], static_cast<double>(point_ranges_[idx]));
          motion_evidence_[idx].residual_inlier = residual_inlier_mask_[idx] != 0;
          motion_evidence_[idx].residual = point_residuals_[idx];
          motion_evidence_[idx].range = static_cast<double>(point_ranges_[idx]);
          motion_evidence_[idx].label = motion_labels_[idx];
          if(!residual_inlier_mask_[idx]) continue;
          
          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;
            double observation_robust_weight = 1.0;
            if (g_observation_robust_kernel != RobustKernel::None) {
              observation_robust_weight = robustObservationWeight(
                  g_observation_robust_kernel, error,
                  g_observation_robust_scale_m);
            }
            if (collect_diagnostics_this_frame_) {
              local_acc.residuals.push_back(static_cast<double>(error));
            }

            const double baseline_omega =
                g_point_to_plane_information * observation_robust_weight;
            if (!std::isfinite(baseline_omega) ||
                baseline_omega <= std::numeric_limits<double>::epsilon()) {
              continue;
            }
            local_acc.baseline_HTVH +=
                J * baseline_omega * J.transpose();
            local_acc.baseline_HTVr -= J * baseline_omega * error;

            if (!g_dynamic_filter_options.enable) {
              local_acc.dynamic_HTVH +=
                  J * baseline_omega * J.transpose();
              local_acc.dynamic_HTVr -= J * baseline_omega * error;
              ++local_acc.dynamic_effective_points;
              continue;
            }

            const double motion_weight =
              dynamic_filter_.labelWeight(motion_labels_[idx]);
            const double omega = baseline_omega * motion_weight;
            if (!std::isfinite(omega) ||
                omega <= std::numeric_limits<double>::epsilon()) {
              continue;
            }
            local_acc.dynamic_HTVH += J * omega * J.transpose();
            local_acc.dynamic_HTVr -= J * omega * error;
            ++local_acc.dynamic_effective_points;
          }
        }
    });

    M6d dynamic_HTVH = M6d::Zero();
    V6d dynamic_HTVr = V6d::Zero();
    M6d baseline_HTVH = M6d::Zero();
    V6d baseline_HTVr = V6d::Zero();
    std::size_t dynamic_effective_points = 0;
    for(const auto& local_acc : tls_acc){
      dynamic_HTVH += local_acc.dynamic_HTVH;
      dynamic_HTVr += local_acc.dynamic_HTVr;
      baseline_HTVH += local_acc.baseline_HTVH;
      baseline_HTVr += local_acc.baseline_HTVr;
      dynamic_effective_points += local_acc.dynamic_effective_points;
    }

    DynamicObservationQuality quality = evaluateDynamicObservationQuality(
        dynamic_HTVH, baseline_HTVH, dynamic_effective_points,
        g_dynamic_filter_options);
    if (!g_dynamic_filter_options.enable) {
      quality.fallback_required = false;
    }
    dynamic_observation_quality_ = quality;
    dynamic_fallback_required_this_frame_ =
        dynamic_fallback_required_this_frame_ || quality.fallback_required;
    const bool dynamic_pose_filtering = g_dynamic_filter_options.enable &&
        g_dynamic_filter_options.filter_pose_update;
    const bool use_fallback =
        dynamic_pose_filtering && quality.fallback_required;
    dynamic_fallback_used_this_frame_ =
        dynamic_fallback_used_this_frame_ || use_fallback;
    tracking_degraded_this_frame_ =
        tracking_degraded_this_frame_ || use_fallback;
    if (dynamic_pose_filtering && !use_fallback) {
      HTVH = dynamic_HTVH.cast<scalar>();
      HTVr = dynamic_HTVr.cast<scalar>();
    } else {
      // Pose-filter bypass, disabled mode, and degraded dynamic mode all use
      // the exact observation assembled by the validated baseline path.
      HTVH = baseline_HTVH.cast<scalar>();
      HTVr = baseline_HTVr.cast<scalar>();
    }

    if (collect_diagnostics_this_frame_) {
      std::vector<double> residuals;
      for (const auto& local_acc : tls_acc) {
        residuals.insert(
            residuals.end(), local_acc.residuals.begin(), local_acc.residuals.end());
      }
      observation_iterations.push_back(ObservationIterationSummary{
          effect_knn_num_, HTVH.template cast<double>(),
          summarizeResiduals(residuals)});
    }

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!knn_valid_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  }, iteration_diagnostic);

  // The scan update rewinds the high-rate nominal state to the scan end. Reset
  // its IMU integration boundary to the exact same timestamp, then replay any
  // later raw IMUs that arrived before the scan callback was serviced.
  const IMUData* imu_at_scan_end = nullptr;
  if (measures_.imu_at_lidar_end) {
    imu_at_scan_end = &*measures_.imu_at_lidar_end;
  } else if (!measures_.imu.empty() &&
             measures_.imu.back().secs == measures_.lidar.end_time) {
    imu_at_scan_end = &measures_.imu.back();
  }
  if (imu_at_scan_end != nullptr) {
    data_wrapper_->synchronizeForwardState(
        *imu_at_scan_end, measures_.imu_after_lidar_end);
  }

  if (collect_diagnostics_this_frame_) {
    const SysState state_after = kf_->GetSysState();
    const ESKF::COV covariance_after = kf_->GetCov();
    const V3 rotation_correction =
        (state_before.R.inverse() * state_after.R).log_vee();
    const V3 position_correction = state_after.p - state_before.p;
    scan_diagnostics_.scan_rotation_correction_rad =
        rotation_correction.norm();
    scan_diagnostics_.scan_position_correction_m =
        position_correction.norm();

    const auto copy_vector3 = [](const V3& source, std::array<double, 3>& target) {
      for (int index = 0; index < 3; ++index) {
        target[index] = source[index];
      }
    };
    const auto copy_quaternion = [](const SO3& source, std::array<double, 4>& target) {
      const V4 coefficients = source.coeffs();
      for (int index = 0; index < 4; ++index) {
        target[index] = coefficients[index];
      }
    };
    copy_vector3(
        rotation_correction, scan_diagnostics_.scan_rotation_correction_vector_rad);
    copy_vector3(
        position_correction, scan_diagnostics_.scan_position_correction_vector_m);
    copy_quaternion(
        state_before.R, scan_diagnostics_.state_orientation_before_xyzw);
    copy_quaternion(
        state_after.R, scan_diagnostics_.state_orientation_after_xyzw);
    copy_vector3(state_before.p, scan_diagnostics_.state_position_before_m);
    copy_vector3(state_after.p, scan_diagnostics_.state_position_after_m);
    copy_vector3(state_before.v, scan_diagnostics_.state_velocity_before_mps);
    copy_vector3(state_after.v, scan_diagnostics_.state_velocity_after_mps);
    copy_vector3(state_before.bg, scan_diagnostics_.state_gyro_bias_before_radps);
    copy_vector3(state_after.bg, scan_diagnostics_.state_gyro_bias_after_radps);
    copy_vector3(state_before.ba, scan_diagnostics_.state_accel_bias_before_mps2);
    copy_vector3(state_after.ba, scan_diagnostics_.state_accel_bias_after_mps2);
    copy_vector3(gravity_before, scan_diagnostics_.state_gravity_before_mps2);
    copy_vector3(kf_->GetGravity(), scan_diagnostics_.state_gravity_after_mps2);

    for (const auto& iteration : filter_iterations) {
      scan_diagnostics_.iteration_rotation_correction_rad.push_back(
          iteration.dx.template head<3>().norm());
      scan_diagnostics_.iteration_position_correction_m.push_back(
          iteration.dx.template segment<3>(3).norm());
      for (int index = 0; index < iteration.dx.rows(); ++index) {
        scan_diagnostics_.iteration_state_correction.push_back(iteration.dx[index]);
      }
    }

    if (!observation_iterations.empty()) {
      const auto& final_observation = observation_iterations.back();
      scan_diagnostics_.knn_candidate_count = final_observation.candidate_count;
      scan_diagnostics_.matched_point_count = final_observation.residual.count;
      scan_diagnostics_.match_fraction = final_observation.candidate_count == 0 ? 0.0 :
          static_cast<double>(final_observation.residual.count) /
          static_cast<double>(final_observation.candidate_count);
      scan_diagnostics_.residual = final_observation.residual;

      Eigen::SelfAdjointEigenSolver<M6d> information_solver(
          0.5 * (final_observation.information +
                 final_observation.information.transpose()));
      if (information_solver.info() == Eigen::Success) {
        const auto eigenvalues = information_solver.eigenvalues();
        const auto eigenvectors = information_solver.eigenvectors();
        const double largest = std::max(0.0, eigenvalues[5]);
        const double rank_threshold = largest * 1e-9;
        for (int index = 0; index < 6; ++index) {
          scan_diagnostics_.information_eigenvalues[index] = eigenvalues[index];
          scan_diagnostics_.information_weakest_direction[index] = eigenvectors(index, 0);
          if (eigenvalues[index] > rank_threshold) {
            ++scan_diagnostics_.information_rank;
          }
        }
        const double condition_floor = largest * 1e-15;
        const double smallest = std::max(eigenvalues[0], condition_floor);
        scan_diagnostics_.information_condition_number =
            smallest > 0.0 ? largest / smallest : 0.0;
      }
    }

    const auto fill_covariance = [](const ESKF::COV& covariance,
                                    std::array<double, 18>& diagonal,
                                    double& minimum_eigenvalue,
                                    double& symmetry_error) {
      const M18d covariance_double = covariance.template cast<double>();
      for (int index = 0; index < 18; ++index) {
        diagonal[index] = covariance_double(index, index);
      }
      symmetry_error =
          (covariance_double - covariance_double.transpose()).cwiseAbs().maxCoeff();
      Eigen::SelfAdjointEigenSolver<M18d> solver(
          0.5 * (covariance_double + covariance_double.transpose()));
      minimum_eigenvalue = solver.info() == Eigen::Success ?
          solver.eigenvalues().minCoeff() : 0.0;
    };
    fill_covariance(
        covariance_before, scan_diagnostics_.covariance_before_diagonal,
        scan_diagnostics_.covariance_before_min_eigenvalue,
        scan_diagnostics_.covariance_before_symmetry_error);
    fill_covariance(
        covariance_after, scan_diagnostics_.covariance_after_diagonal,
        scan_diagnostics_.covariance_after_min_eigenvalue,
        scan_diagnostics_.covariance_after_symmetry_error);
  }

  const SE3 final_pose = kf_->GetSE3();
  static_point_count_ = 0;
  unknown_point_count_ = 0;
  dynamic_candidate_count_ = 0;
  dynamic_confirmed_count_ = 0;
  for (std::size_t idx = 0; idx < ptsize; ++idx) {
    if (plane_valid_mask_[idx]) {
      const V3 point_world = final_pose * points_body_v3_[idx];
      const auto& abcd = abcd_vec_[idx];
      const double residual = abcd[0] * point_world[0] +
          abcd[1] * point_world[1] + abcd[2] * point_world[2] + abcd[3];
      point_residuals_[idx] = residual;
      scalar residual_scalar = static_cast<scalar>(residual);
      residual_inlier_mask_[idx] = compute_error(
        abcd, point_world, point_ranges_[idx], residual_scalar);
      motion_labels_[idx] = dynamic_filter_.classify(
        true, residual, static_cast<double>(point_ranges_[idx]));
      motion_evidence_[idx].residual_inlier = residual_inlier_mask_[idx] != 0;
      motion_evidence_[idx].residual = residual;
      motion_evidence_[idx].range = static_cast<double>(point_ranges_[idx]);
      motion_evidence_[idx].label = motion_labels_[idx];
    } else {
      motion_labels_[idx] = MotionLabel::UNKNOWN;
      motion_evidence_[idx].label = MotionLabel::UNKNOWN;
    }

    switch (motion_labels_[idx]) {
      case MotionLabel::STATIC_CONFIRMED:
        ++static_point_count_;
        break;
      case MotionLabel::DYNAMIC_CANDIDATE:
        ++dynamic_candidate_count_;
        break;
      case MotionLabel::DYNAMIC_CONFIRMED:
        ++dynamic_confirmed_count_;
        break;
      case MotionLabel::UNKNOWN:
      default:
        ++unknown_point_count_;
        break;
    }
  }
  const std::size_t dynamic_points =
      dynamic_candidate_count_ + dynamic_confirmed_count_;
  dynamic_ratio_ = ptsize == 0
      ? 0.0
      : static_cast<double>(dynamic_points) / static_cast<double>(ptsize);

  if (collect_diagnostics_this_frame_) {
    scan_diagnostics_.dynamic_filter_enabled =
        g_dynamic_filter_options.enable;
    scan_diagnostics_.dynamic_pose_filter_enabled =
        g_dynamic_filter_options.enable &&
        g_dynamic_filter_options.filter_pose_update;
    scan_diagnostics_.dynamic_map_filter_enabled =
        g_dynamic_filter_options.enable &&
        g_dynamic_filter_options.filter_map_update;
    scan_diagnostics_.dynamic_static_point_count = static_point_count_;
    scan_diagnostics_.dynamic_unknown_point_count = unknown_point_count_;
    scan_diagnostics_.dynamic_candidate_point_count =
        dynamic_candidate_count_;
    scan_diagnostics_.dynamic_confirmed_point_count =
        dynamic_confirmed_count_;
    scan_diagnostics_.dynamic_ratio = dynamic_ratio_;
    scan_diagnostics_.dynamic_effective_point_count =
        dynamic_observation_quality_.effective_point_count;
    scan_diagnostics_.dynamic_information_ratio =
        dynamic_observation_quality_.information_ratio;
    scan_diagnostics_.dynamic_baseline_information_rank =
        static_cast<std::uint32_t>(
            std::max(0, dynamic_observation_quality_.baseline_rank));
    scan_diagnostics_.dynamic_weighted_information_rank =
        static_cast<std::uint32_t>(
            std::max(0, dynamic_observation_quality_.dynamic_rank));
    scan_diagnostics_.dynamic_fallback_required =
        dynamic_fallback_required_this_frame_;
    scan_diagnostics_.dynamic_fallback_used =
        dynamic_fallback_used_this_frame_;
  }

}


void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  newly_promoted_points_world_.clear();
  if (ptsize == 0) {
    points_world_v3_.clear();
    dynamic_map_update_skipped_this_frame_ =
        g_dynamic_filter_options.enable &&
        g_dynamic_filter_options.filter_map_update;
    if (collect_diagnostics_this_frame_) {
      scan_diagnostics_.dynamic_map_update_skipped =
          dynamic_map_update_skipped_this_frame_;
      scan_diagnostics_.dynamic_temporal_voxel_count =
          temporal_voxel_map_.size();
    }
    return;
  }

  transformCurrentPointsToWorld();
  updateMapFromCurrentWorldPoints();
}


void SuperLIO::transformCurrentPointsToWorld() {
  const std::size_t ptsize = points_body_v3_.size();
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);

  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;

  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
}


void SuperLIO::updateMapFromCurrentWorldPoints() {
  if (!g_dynamic_filter_options.enable ||
      !g_dynamic_filter_options.filter_map_update) {
    ivox_->insert(points_world_v3_);
    if (collect_diagnostics_this_frame_) {
      scan_diagnostics_.dynamic_map_update_skipped = false;
      scan_diagnostics_.dynamic_promoted_point_count = 0;
      scan_diagnostics_.dynamic_temporal_voxel_count =
          temporal_voxel_map_.size();
    }
    return;
  }

  if (tracking_degraded_this_frame_) {
    dynamic_map_update_skipped_this_frame_ = true;
    LOG_EVERY_N(WARNING, 10) << YELLOW
                 << " ---> [DynamicFilter] Tracking fallback active; skip map update for frame "
                 << processing_frame_id_ << RESET;
    if (collect_diagnostics_this_frame_) {
      scan_diagnostics_.dynamic_map_update_skipped = true;
      scan_diagnostics_.dynamic_temporal_voxel_count =
          temporal_voxel_map_.size();
    }
    return;
  }

  if (points_world_v3_.size() != motion_labels_.size()) {
    dynamic_map_update_skipped_this_frame_ = true;
    LOG(WARNING) << YELLOW
                 << " ---> [DynamicFilter] Label/point size mismatch; skip map update."
                 << RESET;
    if (collect_diagnostics_this_frame_) {
      scan_diagnostics_.dynamic_map_update_skipped = true;
      scan_diagnostics_.dynamic_temporal_voxel_count =
          temporal_voxel_map_.size();
    }
    return;
  }

  VV3 static_points_world;
  VV3 unknown_points_world;
  static_points_world.reserve(static_point_count_);
  unknown_points_world.reserve(unknown_point_count_);
  for (std::size_t i = 0; i < points_world_v3_.size(); ++i) {
    switch (motion_labels_[i]) {
      case MotionLabel::STATIC_CONFIRMED:
        static_points_world.push_back(points_world_v3_[i]);
        break;
      case MotionLabel::UNKNOWN:
        unknown_points_world.push_back(points_world_v3_[i]);
        break;
      case MotionLabel::DYNAMIC_CANDIDATE:
      case MotionLabel::DYNAMIC_CONFIRMED:
        break;
    }
  }

  newly_promoted_points_world_ = temporal_voxel_map_.update(
      processing_frame_id_, unknown_points_world);
  static_points_world.insert(static_points_world.end(),
      newly_promoted_points_world_.begin(),
      newly_promoted_points_world_.end());
  ivox_->insert(static_points_world);

  if (collect_diagnostics_this_frame_) {
    scan_diagnostics_.dynamic_map_update_skipped = false;
    scan_diagnostics_.dynamic_promoted_point_count =
        newly_promoted_points_world_.size();
    scan_diagnostics_.dynamic_temporal_voxel_count =
        temporal_voxel_map_.size();
  }

}


void SuperLIO::publishDynamicDebugClouds() {
  if (!g_dynamic_filter_options.enable ||
      !g_dynamic_filter_options.publish_debug_clouds) {
    return;
  }
  if (processing_frame_id_ % static_cast<std::uint64_t>(
          g_dynamic_filter_options.debug_publish_every_n_frames) != 0) {
    return;
  }
  if (points_world_v3_.size() != motion_labels_.size() ||
      ds_undistort_->size() != motion_labels_.size()) {
    LOG(WARNING) << YELLOW
                 << " ---> [DynamicFilter] Label/point size mismatch; skip debug clouds."
                 << RESET;
    return;
  }

  CloudPtr static_cloud(new PointCloudType());
  CloudPtr dynamic_cloud(new PointCloudType());
  CloudPtr unknown_cloud(new PointCloudType());
  static_cloud->reserve(static_point_count_);
  dynamic_cloud->reserve(
      dynamic_candidate_count_ + dynamic_confirmed_count_);
  unknown_cloud->reserve(unknown_point_count_);

  for (std::size_t i = 0; i < motion_labels_.size(); ++i) {
    PointType point = ds_undistort_->points[i];
    point.x = points_world_v3_[i][0];
    point.y = points_world_v3_[i][1];
    point.z = points_world_v3_[i][2];
    switch (motion_labels_[i]) {
      case MotionLabel::STATIC_CONFIRMED:
        static_cloud->push_back(point);
        break;
      case MotionLabel::DYNAMIC_CANDIDATE:
      case MotionLabel::DYNAMIC_CONFIRMED:
        dynamic_cloud->push_back(point);
        break;
      case MotionLabel::UNKNOWN:
      default:
        unknown_cloud->push_back(point);
        break;
    }
  }

  data_wrapper_->pub_dynamic_debug_clouds(
      static_cloud, dynamic_cloud, unknown_cloud,
      measures_.lidar.end_time);
}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state);
  data_wrapper_->pub_dynamic_ratio(static_cast<float>(dynamic_ratio_));
  publishDynamicDebugClouds();

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  CloudPtr world_pc(new PointCloudType());
  
  if(g_visual_map){
    static int count = -1;
    count++;
    if(count % g_pub_step != 0){
      return;
    }
    count = 0;
    if(g_visual_dense){
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }else{
      pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }
  }
}

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

} // namespace END.
