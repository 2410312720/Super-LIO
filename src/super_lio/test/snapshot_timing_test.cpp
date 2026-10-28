// Regression tests for explicit Gazebo mode and real-scan compatibility.
#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "lio/super_lio.h"

using namespace BASIC;
using namespace LI2Sup;

namespace {

IMUData imu(double time, double yaw_rate = 0.0, double accel_x = 0.0) {
  IMUData result;
  result.secs = time;
  result.gyr = V3(0, 0, yaw_rate);
  result.acc = V3(accel_x, 0, 9.81);
  return result;
}

// Exercise the production propagation and initialization paths directly.
class TestLIO : public SuperLIO {
public:
  TestLIO() {
    g_gravity_norm = 9.81;
    kf_ = std::make_shared<ESKF>();
    kf_->SetInitialConditions(ESKF::Options(), V3::Zero(), V3::Zero());
    scan_undistort_full_.reset(new PointCloudType());
  }
  void seed(const IMUData& first, const V3& velocity = V3::Zero()) {
    SysState state(first.secs);
    state.v = velocity;
    kf_->SetX(state);
    kf_->Predict(first);
  }
  using SuperLIO::Propagation_Undistort;
  using SuperLIO::map_init;
  using SuperLIO::measures_;
  using SuperLIO::scan_undistort_full_;
  using SuperLIO::propagate_states_;
  using SuperLIO::kf_;
  using SuperLIO::ivox_;
  using SuperLIO::OctVoxMapType;
};

TEST(Propagation, SnapshotPreservesGeometryWhilePredictingYawAndTranslation) {
  for (const double yaw_rate : {0.0, 6.0, -12.0}) {
    TestLIO lio;
    g_lidar_imu = SE3(SO3::Exp(V3(0.2, -0.1, 0.3)), V3(-0.011, -0.02329, 0.04412));
    lio.seed(imu(10.0, yaw_rate, 2.0), V3(1, 0, 0));
    auto& meas = lio.measures_;
    meas.lidar.start_time = meas.lidar.end_time = 10.005;
    meas.lidar.is_snapshot = true;
    meas.lidar.pc.reset(new pcl::PointCloud<PointXTZIT>());
    for (int i = 0; i < 100; ++i) {
      meas.lidar.pc->emplace_back(1.0 + 0.13 * i, 0.1 * (i % 7), 0.2 * (i % 11), i, 0.0);
    }
    meas.imu = {imu(10.0, yaw_rate, 2.0), imu(10.005, yaw_rate, 2.0)};
    lio.Propagation_Undistort();
    EXPECT_DOUBLE_EQ(lio.kf_->GetTime(), meas.lidar.end_time);
    EXPECT_NEAR(lio.kf_->GetSysState().p.x(), 0.005025, 1e-7);
    EXPECT_NEAR(lio.kf_->GetSysState().R.log_vee().z(), yaw_rate * 0.005, 1e-6);
    ASSERT_EQ(lio.propagate_states_.size(), 2u);
    double max_residual = 0.0;
    for (size_t i = 0; i < meas.lidar.pc->size(); ++i) {
      const auto& raw = meas.lidar.pc->points[i];
      const auto& processed = lio.scan_undistort_full_->points[i];
      const V3 expected = g_lidar_imu * V3(raw.x, raw.y, raw.z);
      max_residual = std::max(max_residual,
          static_cast<double>((expected - V3(processed.x, processed.y, processed.z)).norm()));
      EXPECT_FLOAT_EQ(processed.intensity, raw.intensity);
    }
    EXPECT_LT(max_residual, 2e-6);
  }
}

TEST(Propagation, RealScanUsesPointTimeAndHandlesEndpointsAndDuplicateImu) {
  TestLIO lio;
  g_lidar_imu = SE3();
  lio.seed(imu(20.0, 0.0, 2.0), V3(1, 0, 0));
  auto& meas = lio.measures_;
  meas.lidar.start_time = 20.0;
  meas.lidar.end_time = 20.1;
  meas.lidar.pc.reset(new pcl::PointCloud<PointXTZIT>());
  const std::vector<double> offsets{-0.01, 0.0, 0.025, 0.05, 0.1, 0.11};
  for (double offset : offsets) meas.lidar.pc->emplace_back(5, 0, 0, 1, offset);
  meas.imu = {imu(20.0, 0, 2), imu(20.05, 0, 2), imu(20.05, 0, 2),
              imu(20.04, 0, 2), imu(20.1, 0, 2)};
  lio.Propagation_Undistort();
  ASSERT_EQ(lio.propagate_states_.size(), 3u);
  EXPECT_DOUBLE_EQ(lio.kf_->GetTime(), 20.1);
  EXPECT_TRUE(lio.kf_->GetCov().allFinite());
  for (size_t i = 0; i < offsets.size(); ++i) {
    const double tau = std::clamp(offsets[i], 0.0, 0.1);
    EXPECT_NEAR(lio.scan_undistort_full_->points[i].x,
                5.0 + tau + tau * tau - 0.11, 2e-6);
  }
  // A single-state history must also be safe at both boundaries.
  meas.imu.clear();
  lio.Propagation_Undistort();
  ASSERT_EQ(lio.propagate_states_.size(), 1u);
  for (const auto& point : *lio.scan_undistort_full_) EXPECT_NEAR(point.x, 5.0, 2e-6);
}

TEST(Propagation, MapInitializationAdvancesStateToCloudTime) {
  TestLIO lio;
  g_lidar_imu = SE3();
  lio.seed(imu(30.0, 4.0));
  lio.ivox_ = std::make_shared<TestLIO::OctVoxMapType>(
      TestLIO::OctVoxMapType::Options{0.5, 1000});
  auto& meas = lio.measures_;
  meas.lidar.start_time = meas.lidar.end_time = 30.01;
  meas.lidar.is_snapshot = true;
  meas.lidar.pc.reset(new pcl::PointCloud<PointXTZIT>());
  meas.lidar.pc->emplace_back(5, 0, 0, 1, 0);
  meas.imu = {imu(30.0, 4), imu(30.01, 4)};
  lio.map_init();
  EXPECT_DOUBLE_EQ(lio.kf_->GetTime(), 30.01);
  EXPECT_NEAR(lio.kf_->GetSysState().R.log_vee().z(), 0.04, 1e-6);
}

// Use the public ROS interface so the test covers PointCloud2 conversion,
// callback timestamp checks, synchronization, and published pose timestamps.
class SnapshotSync : public ::testing::Test {
protected:
  virtual void configureMode(rclcpp::NodeOptions& options) {
    options.append_parameter_override("lio.sensor.is_gazebo", true);
  }
  void SetUp() override {
    rclcpp::init(0, nullptr);
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("lio.sensor.lidar_type", 8),
        rclcpp::Parameter("lio.ros.lidar_topic", "/snapshot_test/cloud"),
        rclcpp::Parameter("lio.ros.imu_topic", "/snapshot_test/imu")});
    configureMode(options);
    wrapper = std::make_shared<ROSWrapper>(options);
    auto filter = std::make_shared<ESKF>();
    wrapper->setESKF(filter);
    source = std::make_shared<rclcpp::Node>("snapshot_test_source");
    if (g_lidar_type != LID_TYPE::LIVOX) {
      cloud_pub = source->create_publisher<sensor_msgs::msg::PointCloud2>("/snapshot_test/cloud", 20);
    }
    imu_pub = source->create_publisher<sensor_msgs::msg::Imu>("/snapshot_test/imu", 100);
    odom_sub = source->create_subscription<nav_msgs::msg::Odometry>("/lio/odom", 10,
        [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom = msg; });
    executor->add_node(wrapper);
    executor->add_node(source);
    ASSERT_TRUE(spinUntil([&] {
      return (g_lidar_type == LID_TYPE::LIVOX || cloud_pub->get_subscription_count() > 0) &&
          imu_pub->get_subscription_count() > 0;
    }));
  }
  void TearDown() override {
    executor.reset();
    odom_sub.reset(); cloud_pub.reset(); imu_pub.reset(); wrapper.reset(); source.reset();
    rclcpp::shutdown();
  }
  template<class Predicate> bool spinUntil(Predicate done) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    do {
      executor->spin_some();
      if (done()) return true;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < deadline);
    return false;
  }
  void sendImu(int64_t ns, double yaw) {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(ns);
    msg.linear_acceleration.x = yaw * 2;
    msg.linear_acceleration.z = 9.81;
    msg.angular_velocity.z = yaw;
    imu_pub->publish(msg);
  }
  void sendCloud(int64_t ns) {
    pcl::PointCloud<LivoxSIM::Point> cloud;
    for (int i = 0; i < 12; ++i) {
      LivoxSIM::Point p{};
      p.x = 5 + i; p.y = 1; p.z = 2; p.intensity = i;
      cloud.push_back(p);
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = rclcpp::Time(ns);
    cloud_pub->publish(msg);
  }
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  ROSWrapper::Ptr wrapper;
  rclcpp::Node::SharedPtr source;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  nav_msgs::msg::Odometry::SharedPtr odom;
};

TEST_F(SnapshotSync, InterpolatesBoundaryRetainsFutureSampleAndPublishesAtSampleTime) {
  constexpr int64_t second = 1000000000;
  sendImu(second, 2);
  sendImu(second, 99);  // A duplicate must not change the interpolation bracket.
  sendCloud(second + 2500000);
  // Process callbacks while the future bracket is still unavailable.
  MeasureGroup meas;
  const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
  ASSERT_TRUE(spinUntil([&] {
    EXPECT_FALSE(wrapper->sync_measure(meas));
    return std::chrono::steady_clock::now() >= until;
  }));
  sendImu(second + 10000000, 6);
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  EXPECT_TRUE(meas.lidar.is_snapshot);
  EXPECT_DOUBLE_EQ(meas.lidar.start_time, meas.lidar.end_time);
  ASSERT_EQ(meas.lidar.pc->size(), 12u);
  for (const auto& p : *meas.lidar.pc) EXPECT_DOUBLE_EQ(p.offset_time, 0.0);
  ASSERT_EQ(meas.imu.size(), 2u);
  EXPECT_DOUBLE_EQ(meas.imu.back().secs, meas.lidar.end_time);
  EXPECT_NEAR(meas.imu.back().gyr.z(), 3, 1e-6);
  EXPECT_NEAR(meas.imu.back().acc.x(), 6, 1e-6);

  TestLIO lio;
  g_lidar_imu = SE3();
  lio.seed(meas.imu.front());
  lio.measures_ = meas;
  lio.Propagation_Undistort();
  wrapper->pub_odom(lio.kf_->GetNavState());
  ASSERT_TRUE(spinUntil([&] { return odom != nullptr; }));
  EXPECT_NEAR(rclcpp::Time(odom->header.stamp).seconds(), meas.lidar.end_time, 1e-9);
  EXPECT_NEAR(lio.kf_->GetSysState().R.log_vee().z(), 0.00625, 1e-7);

  sendCloud(second + 7500000);  // Two cloud times inside the same IMU interval.
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  ASSERT_EQ(meas.imu.size(), 2u);
  EXPECT_DOUBLE_EQ(meas.imu.front().secs, lio.kf_->GetTime());
  EXPECT_NEAR(meas.imu.back().gyr.z(), 5, 1e-6);
  lio.measures_ = meas;
  lio.Propagation_Undistort();
  EXPECT_NEAR(lio.kf_->GetSysState().R.log_vee().z(), 0.02625, 1e-7);

  sendCloud(second + 10000000);  // The original later IMU is still available.
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  EXPECT_DOUBLE_EQ(meas.imu.back().secs, meas.lidar.end_time);
  EXPECT_FLOAT_EQ(meas.imu.back().gyr.z(), 6);
}

TEST_F(SnapshotSync, DropsCloudsBeforeImuHistoryAndDuplicateCloudTimes) {
  constexpr int64_t second = 1000000000;
  sendImu(second, 2);
  sendImu(second + 10000000, 6);
  sendCloud(second - 1000000);
  sendCloud(second);
  MeasureGroup meas;
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  EXPECT_DOUBLE_EQ(meas.lidar.end_time, 1.0);
  ASSERT_EQ(meas.imu.size(), 1u);
  sendCloud(second);
  sendCloud(second + 5000000);
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  EXPECT_DOUBLE_EQ(meas.lidar.end_time, 1.005);
  EXPECT_NEAR(meas.imu.back().gyr.z(), 4, 1e-6);
}


// Omitting the parameter must preserve the previous scan semantics.
class LegacySync : public SnapshotSync {
protected:
  void configureMode(rclcpp::NodeOptions&) override {}
};

TEST_F(LegacySync, DefaultModeKeepsIndexedTimesAndOriginalImuBatching) {
  EXPECT_FALSE(wrapper->get_parameter("lio.sensor.is_gazebo").as_bool());
  sendImu(1000000000, 2);
  sendImu(1090000000, 4);
  sendImu(1200000000, 6);
  sendCloud(1000000000);
  MeasureGroup meas;
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  EXPECT_FALSE(meas.lidar.is_snapshot);
  EXPECT_NEAR(meas.lidar.end_time, 1.1, 1e-9);
  EXPECT_NEAR(meas.lidar.pc->points.back().offset_time, 0.1, 1e-9);
  ASSERT_EQ(meas.imu.size(), 2u);
  EXPECT_DOUBLE_EQ(meas.imu.back().secs, 1.09);  // No synthetic endpoint in real mode.

  sendImu(1300000000, 8);
  sendImu(1500000000, 10);
  sendCloud(1200000000);
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  EXPECT_NEAR(meas.lidar.pc->points.back().offset_time, 0.2, 1e-9);
  ASSERT_EQ(meas.imu.size(), 2u);
  EXPECT_DOUBLE_EQ(meas.imu.front().secs, 1.2);  // Later IMU from the first batch.
  EXPECT_DOUBLE_EQ(meas.imu.back().secs, 1.3);
}

class RealLivoxSync : public SnapshotSync {
protected:
  void configureMode(rclcpp::NodeOptions& options) override {
    options.append_parameter_override("lio.sensor.is_gazebo", false);
    // Replace the format override already supplied by the base fixture.
    auto parameters = options.parameter_overrides();
    for (auto& parameter : parameters) {
      if (parameter.get_name() == "lio.sensor.lidar_type") {
        parameter = rclcpp::Parameter("lio.sensor.lidar_type", 1);
      }
    }
    options.parameter_overrides(parameters);
  }
};

TEST_F(RealLivoxSync, DisabledGazeboKeepsHardwarePointTimestamps) {
  auto publisher = source->create_publisher<livox_ros_driver2::msg::CustomMsg>(
      "/snapshot_test/cloud", 20);
  ASSERT_TRUE(spinUntil([&] { return publisher->get_subscription_count() > 0; }));
  livox_ros_driver2::msg::CustomMsg cloud;
  cloud.header.stamp = rclcpp::Time(int64_t{1000000000});
  cloud.point_num = 12;
  cloud.points.resize(cloud.point_num);
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    cloud.points[i].x = 5 + i;
    cloud.points[i].reflectivity = i;
    cloud.points[i].offset_time = i * 1000000;
  }
  sendImu(1000000000, 2);
  sendImu(1010000000, 4);
  sendImu(1020000000, 6);
  publisher->publish(cloud);
  MeasureGroup meas;
  ASSERT_TRUE(spinUntil([&] { return wrapper->sync_measure(meas); }));
  EXPECT_FALSE(meas.lidar.is_snapshot);
  ASSERT_EQ(meas.lidar.pc->size(), cloud.points.size());
  for (size_t i = 0; i < cloud.points.size(); ++i) {
    EXPECT_NEAR(meas.lidar.pc->points[i].offset_time, i * 0.001, 1e-9);
  }
  EXPECT_NEAR(meas.lidar.end_time, 1.011, 1e-9);
  EXPECT_DOUBLE_EQ(meas.imu.back().secs, 1.01);
}

TEST(Propagation, RealScanKeepsOriginalMapInitialization) {
  TestLIO lio;
  g_lidar_imu = SE3();
  lio.seed(imu(30.0, 4.0));
  lio.ivox_ = std::make_shared<TestLIO::OctVoxMapType>(
      TestLIO::OctVoxMapType::Options{0.5, 1000});
  auto& meas = lio.measures_;
  meas.lidar.start_time = 30.0;
  meas.lidar.end_time = 30.01;
  meas.lidar.pc.reset(new pcl::PointCloud<PointXTZIT>());
  meas.lidar.pc->emplace_back(5, 0, 0, 1, 0);
  meas.imu = {imu(30.0, 4), imu(30.01, 4)};
  lio.map_init();
  EXPECT_DOUBLE_EQ(lio.kf_->GetTime(), 30.0);
  EXPECT_TRUE(lio.scan_undistort_full_->empty());
}

}  // namespace
