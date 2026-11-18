#include <chrono>
#include <cmath>

#include <gtest/gtest.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ros/ROSWrapper.h"

namespace {

class BodyTfTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void verify(const std::string& world, const std::string& imu,
              const std::string& body, bool with_extrinsic) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("lio.sensor.lidar_type", 6),
        rclcpp::Parameter("lio.output.world_frame_id", world),
        rclcpp::Parameter("lio.output.imu_frame_id", imu),
        rclcpp::Parameter("lio.output.robot_frame_id", body),
        rclcpp::Parameter("lio.extrinsic.odom_robo", with_extrinsic ?
            std::vector<double>{1, 0, 0, 0, 0, 90} : std::vector<double>(6, 0))});
    const auto wrapper = std::make_shared<LI2Sup::ROSWrapper>(options);
    const auto observer = std::make_shared<rclcpp::Node>("body_tf_test_observer");
    tf2_ros::Buffer buffer(observer->get_clock());
    tf2_ros::TransformListener listener(buffer, observer, false);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(wrapper);
    executor.add_node(observer);

    // The IMU faces +Y at (10, 20, 3). With a +90-degree robot-to-IMU
    // mounting and a one-metre lever arm the body is at (9, 20, 3), facing +X.
    const BASIC::M3 yaw90 = Eigen::AngleAxis<BASIC::scalar>(
        M_PI / 2, BASIC::V3::UnitZ()).toRotationMatrix();
    const LI2Sup::NavState state(5.5, BASIC::SO3(yaw90), BASIC::V3(10, 20, 3));
    const auto timer = wrapper->create_wall_timer(std::chrono::milliseconds(20),
        [&] { wrapper->pub_odom(state); });
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (std::chrono::steady_clock::now() < deadline &&
           !(buffer.canTransform(world, imu, tf2::TimePointZero) &&
             buffer.canTransform(world, body, tf2::TimePointZero))) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    ASSERT_TRUE(buffer.canTransform(world, imu, tf2::TimePointZero));
    ASSERT_TRUE(buffer.canTransform(world, body, tf2::TimePointZero))
        << "Missing aircraft TF: " << world << " -> " << body;
    const auto imu_tf = buffer.lookupTransform(world, imu, tf2::TimePointZero);
    const auto body_tf = buffer.lookupTransform(world, body, tf2::TimePointZero);
    EXPECT_EQ(body_tf.header.stamp, imu_tf.header.stamp);
    EXPECT_EQ(body_tf.header.stamp.sec, 5);
    EXPECT_EQ(body_tf.header.stamp.nanosec, 500000000U);
    EXPECT_NEAR(imu_tf.transform.translation.x, 10, 1e-5);
    EXPECT_NEAR(imu_tf.transform.translation.y, 20, 1e-5);
    EXPECT_NEAR(imu_tf.transform.rotation.z, std::sqrt(0.5), 1e-5);
    EXPECT_NEAR(body_tf.transform.translation.x, with_extrinsic ? 9 : 10, 1e-5);
    EXPECT_NEAR(body_tf.transform.translation.y, 20, 1e-5);
    EXPECT_NEAR(body_tf.transform.translation.z, 3, 1e-5);
    EXPECT_NEAR(body_tf.transform.rotation.x, 0, 1e-5);
    EXPECT_NEAR(body_tf.transform.rotation.y, 0, 1e-5);
    EXPECT_NEAR(body_tf.transform.rotation.z, with_extrinsic ? 0 : std::sqrt(0.5), 1e-5);
    EXPECT_NEAR(body_tf.transform.rotation.w, with_extrinsic ? 1 : std::sqrt(0.5), 1e-5);
  }
};

TEST_F(BodyTfTest, PublishesDefaultAircraftFrameAlongsideImu) {
  verify("world", "imu", "base_link", false);
}

TEST_F(BodyTfTest, AppliesMountingExtrinsicAndConfiguredFrames) {
  verify("map", "sensor", "aircraft", true);
}

}  // namespace
