
#ifndef ROSWRAPPER_HPP_
#define ROSWRAPPER_HPP_

#include <map>
#include <tuple>
#include <deque>
#include <vector>
#include <execution>
#include <functional>
#include <chrono>
#include <string>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/callback_group.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

/// msgs
#include "super_lio/msg/cloud_pose.hpp"
#include "super_lio/msg/cloud_pose2.hpp"
#include "super_lio/msg/scan_diagnostics.hpp"


#include "lio/params.h"
#include "basic/alias.h"
#include "basic/logs.h"
#include "basic/Manifold.h"
#include "common/ds.h"
#include "common/diagnostics.h"

#include "lio/ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"


namespace LI2Sup{

void LoadParamFromRos(rclcpp::Node& node);

void livox2pcl(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg, BASIC::CloudPtr& point_cloud);

class ROSWrapper : public rclcpp::Node {
public:
  explicit ROSWrapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~ROSWrapper(){};
  using Ptr = std::shared_ptr<ROSWrapper>;
  bool sync_measure(MeasureGroup&);

  void setESKF(ESKF::Ptr& eskf) { eskf_ = eskf;}
  void setMeasurementReadyCallback(std::function<void()> callback) {
    measurement_ready_callback_ = std::move(callback);
  }

  void clear(){
    lidar_buffer_.clear();
    imu_buffer_.clear();
    pending_forward_odometry_.clear();
    lidar_pushed_ = false;
    last_timestamp_imu_ = -1.0;
    last_timestamp_lidar_ = -1.0;
    last_lidar_msg_stamp_ = -1.0;
  }

  void pub_odom(const NavState&);
  void pub_cloud_world(const BASIC::CloudPtr& pc, double time);
  void pub_dynamic_debug_clouds(
      const BASIC::CloudPtr& static_cloud,
      const BASIC::CloudPtr& dynamic_cloud,
      const BASIC::CloudPtr& unknown_cloud,
      double time);
  void pub_dynamic_ratio(float dynamic_ratio);
  void pub_cloud2planner(const BASIC::CloudPtr& pc, double time);
  void pub_cloud_world_pose(const BASIC::CloudPtr& pc, 
                            const NavState& state);
  void pub_cloud_body_pose(const BASIC::CloudPtr& pc, 
                           const NavState& state);
  void pub_cloud_body_pose( const BASIC::VV3& pc_body,
                            const NavState& state);  
  void pub_processing_time(double time, double current_time, double mean_time, double std_time);
  void pub_scan_diagnostics(const ScanDiagnosticsData& diagnostics);
  void synchronizeForwardState(
      const IMUData& imu_at_scan_end,
      const std::deque<IMUData>& imu_after_scan_end);

  void set_global_map(const BASIC::CloudPtr& global_map);

  void set_initial_data(BASIC::SE3& init_pose, bool& flg_get_init_guess, bool flg_finish_init = false);

  rclcpp::CallbackGroup::SharedPtr getSensorCallbackGroup() {
    return cb_sensor_;
  }

private:
  void imuHandler(const sensor_msgs::msg::Imu::SharedPtr msg);
  void livoxHandler(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void stdMsgHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void setupParams();
  void setupIO();
  void notifyMeasurementReady();
  void enqueueForwardOdometry(
      const sensor_msgs::msg::Imu& source,
      const DynamicState& imu_state,
      const DynamicState& robot_state);
  void updatePendingForwardOdometry(
      double timestamp,
      const DynamicState& imu_state,
      const DynamicState& robot_state);
  void flushPendingForwardOdometryThrough(double imu_timestamp);
  void flushPendingForwardOdometryAfterIdle();

private:
  rclcpp::CallbackGroup::SharedPtr cb_sensor_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_std_;

  std::deque<IMUData>   imu_buffer_;
  std::deque<LidarData> lidar_buffer_;
  bool lidar_pushed_ = false;
  double last_timestamp_imu_ = -1.0;
  double last_timestamp_lidar_ = -1.0;
  double last_lidar_msg_stamp_ = -1.0;

  ESKF::Ptr eskf_{nullptr};
  std::function<void()> measurement_ready_callback_;

  struct PendingForwardOdometry {
    double timestamp = 0.0;
    nav_msgs::msg::Odometry imu;
    nav_msgs::msg::Odometry robot;
  };
  std::deque<PendingForwardOdometry> pending_forward_odometry_;
  std::chrono::steady_clock::time_point last_imu_arrival_;
  rclcpp::TimerBase::SharedPtr forward_output_timer_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string robot_frame_id_;

  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PoseStamped msg2uav_;
  sensor_msgs::msg::PointCloud2 global_map_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  BASIC::V3 last_path_point_ = BASIC::V3(0, 0, -100);

/// output.
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;       /// lidar fre --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_imu_odom_;   /// IMU fre   --> IMU frame
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_robo_odom_;  /// IMU fre   --> Robot frame
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_world_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_static_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_dynamic_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_unknown_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_dynamic_ratio_;
  rclcpp::Publisher<super_lio::msg::ScanDiagnostics>::SharedPtr pub_scan_diagnostics_;
};

} // namespace END.

#endif
